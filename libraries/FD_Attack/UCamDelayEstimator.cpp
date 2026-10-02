#include "UCamDelayEstimator.h"

void UCamDelayEstimator::init(float d0_ms)
{
    _d_s = constrain_float(d0_ms, D_MIN_S * 1000.0f, D_MAX_S * 1000.0f) * 0.001f;
    _d_commit = _d_s;        // 初值即回滚基准（首次机动冻结回滚到 UATK_LPF_DLY）
    _clean_frames = 0;
    _scatter_deg = 0.0f;
    _last_t_ms = 0;
    _count = 0;
    _head = 0;
    _n_seen = 0;
    _gyro_ema.zero();
}

bool UCamDelayEstimator::synth_los_deg(const Vector2f &target_c_deg, uint32_t t_att_ms,
                                       const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body,
                                       Vector2f &z_deg) const
{
    Vector3f att_deg;
    if (!attitude_delay.get_at_time_angle(t_att_ms, att_deg)) {
        return false;
    }
    // 相机系单位向量 → 惯性系 → 方位/俯仰角（与 FD_LPF 链同口径）
    const float yaw_c_rad = radians(target_c_deg.x);
    const float pitch_c_rad = radians(target_c_deg.y);
    const Vector3f los_c_unit(cosf(pitch_c_rad) * cosf(yaw_c_rad),
                              cosf(pitch_c_rad) * sinf(yaw_c_rad),
                              -sinf(pitch_c_rad));
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(att_deg.x), radians(att_deg.y), radians(att_deg.z));
    Vector3f los_e_unit = (rotation_ned_from_body * rot_cam_to_body) * los_c_unit;
    los_e_unit.normalize();
    z_deg.x = wrap_180(degrees(atan2f(los_e_unit.y, los_e_unit.x)));
    z_deg.y = degrees(atan2f(-los_e_unit.z, los_e_unit.xy().length()));
    return true;
}

void UCamDelayEstimator::detrend(const float *in, float *out, uint8_t n)
{
    // 最小二乘拟合直线 a+b·k 并减去
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint8_t k = 0; k < n; k++) {
        sx += k;
        sy += in[k];
        sxx += (float)k * k;
        sxy += (float)k * in[k];
    }
    const float det = (float)n * sxx - sx * sx;
    if (fabsf(det) < 1e-6f) {
        for (uint8_t k = 0; k < n; k++) {
            out[k] = in[k] - sy / n;
        }
        return;
    }
    const float b = ((float)n * sxy - sx * sy) / det;
    const float a = (sy - b * sx) / n;
    for (uint8_t k = 0; k < n; k++) {
        out[k] = in[k] - (a + b * k);
    }
}

void UCamDelayEstimator::update(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                                const UDelay &attitude_delay, const Vector3f &gyro_b_dps,
                                const Matrix3f &rot_cam_to_body)
{
    // 机动识别（逐分量慢攻快放 EMA）：瞬时值大于 EMA 说明机动在持续/增强 → 慢攻确认，
    // 防止 1.5Hz 零均值晃动误触发；瞬时值回落 → 快放，机动结束后尽快恢复估计
    for (uint8_t i = 0; i < 3; i++) {
        const float g = gyro_b_dps[i];
        const float a = (fabsf(g) > fabsf(_gyro_ema[i])) ? EMA_ATTACK : EMA_RELEASE;
        _gyro_ema[i] += a * (g - _gyro_ema[i]);
    }

    _n_seen++;

    // 帧间隔（相机帧率会抖动）——只用于 Δ̂ 速率限幅折算
    const uint32_t dt_frame_ms = (_last_t_ms == 0) ? 33U : (t_arr_ms - _last_t_ms);
    _last_t_ms = t_arr_ms;
    const float dt_frame_s = constrain_float((float)dt_frame_ms * 0.001f, 0.005f, 0.200f);

    if (_n_seen <= TRANSIENT_FRAMES) {
        return;   // 门控一：启动暂态不参与
    }

    if (_gyro_ema.length() > MANEUVER_DPS) {
        // 门控二：单向机动段 ω 近似直流，去趋势后仍污染回归——清空窗口并冻结 Δ̂。
        // 冻结前先回滚到"上一个连续干净窗口"确认过的值：EMA 确认机动要 ~0.3s，
        // 这段污染帧已入窗，回滚把污染整段丢掉。
        _d_s = _d_commit;
        _clean_frames = 0;
        _count = 0;
        _head = 0;
        return;
    }

    // 连续一整窗（COMMIT_FRAMES）未被机动打断，才把当前 Δ̂ 提交为回滚基准——
    // 提交门槛挡住机动起始 ~0.3s 的污染段
    if (_clean_frames < COMMIT_FRAMES) {
        _clean_frames++;
    }
    if (_clean_frames >= COMMIT_FRAMES) {
        _d_commit = _d_s;
    }

    _buf[_head] = {target_c_deg, t_arr_ms};
    _head = (_head + 1) % WIN;
    if (_count < WIN) {
        _count++;
    }
    if (_count < MIN_FRAMES) {
        return;
    }

    // 用当前 Δ̂ 重放窗口内所有帧（从老到新），合成 z 并数值扰动求有效角速度 ω_eff
    float zy[WIN], zp[WIN], oy[WIN], op[WIN];
    uint8_t n = 0;
    const uint32_t d_ms  = (uint32_t)(_d_s * 1000.0f);
    const uint32_t dp_ms = d_ms + (uint32_t)(PERTURB_S * 1000.0f);
    for (uint8_t k = 0; k < _count; k++) {
        const uint8_t idx = (_head + WIN - _count + k) % WIN;
        const RawFrame &f = _buf[idx];
        Vector2f z0, z1;
        if (!synth_los_deg(f.target_c_deg, f.t_arr_ms - d_ms, attitude_delay, rot_cam_to_body, z0)) {
            continue;   // 姿态缓冲未覆盖（启动早期/跨度不足），跳过该帧
        }
        if (!synth_los_deg(f.target_c_deg, f.t_arr_ms - dp_ms, attitude_delay, rot_cam_to_body, z1)) {
            continue;
        }
        // 窗口内 yaw 解缠：以本窗第一帧为基准按最短路径一次性展开（pitch 无回绕不动）。
        // 注意这不是逐帧累积 unwrap——窗口 ~1s 内方位角扫过 360° 以上才会混叠，
        // 远景缓变工况碰不到；若将来换更长窗口或目标扫掠更快，要改成逐帧 unwrap。
        zy[n] = (n == 0) ? z0.x : (zy[0] + wrap_180(z0.x - zy[0]));
        zp[n] = z0.y;
        // ω_eff = (z(Δ̂) − z(Δ̂+ε))/ε = 姿态贡献的角速度，符号与 z ≈ θ + ω·(Δ−Δ̂) 一致
        oy[n] = wrap_180(z0.x - z1.x) / PERTURB_S;
        op[n] = (z0.y - z1.y) / PERTURB_S;
        n++;
    }
    if (n < MIN_FRAMES) {
        return;
    }

    // 窗口去趋势（z 与 ω 各自去线性趋势），偏航/俯仰两通道混合回归
    float zyc[WIN], zpc[WIN], oyc[WIN], opc[WIN];
    detrend(zy, zyc, n);
    detrend(zp, zpc, n);
    detrend(oy, oyc, n);
    detrend(op, opc, n);
    float num = 0.0f, den = 0.0f;
    for (uint8_t k = 0; k < n; k++) {
        num += zyc[k] * oyc[k] + zpc[k] * opc[k];
        den += oyc[k] * oyc[k] + opc[k] * opc[k];
    }
    if (den < 2.0f * (float)n * EXCITE_DPS * EXCITE_DPS) {
        // 门控三：激励不足（没在晃），Δ 不可观测也不危害，不更新；散度同时清零表示"不可测"
        _scatter_deg = 0.0f;
        return;
    }

    const float delta = num / den;   // deg/(deg/s) = s，即 Δ_true − Δ̂ 的最小二乘估计

    // 量测散度：扣掉去趋势项与 δ̂·ω 项后残留的逐帧角度散度 RMS
    float ss = 0.0f;
    for (uint8_t k = 0; k < n; k++) {
        const float ry = zyc[k] - delta * oyc[k];
        const float rp = zpc[k] - delta * opc[k];
        ss += ry * ry + rp * rp;
    }
    _scatter_deg = sqrtf(ss / (2.0f * (float)n));

    if (fabsf(delta) < DEADBAND_S) {
        return;
    }
    // Δ̂ 速率限幅：单帧修正按本帧间隔折算，上限 40ms/s——防止任一脏窗一帧把 Δ̂ 推到钳位
    const float step_max = D_RATE_MAX * dt_frame_s;
    const float step = constrain_float(GAIN * delta, -step_max, step_max);
    _d_s = constrain_float(_d_s + step, D_MIN_S, D_MAX_S);
}
