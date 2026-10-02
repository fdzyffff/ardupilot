#include "FD_EKF.h"

// 参数组 UATK_EKF_*（EN 带 AP_PARAM_FLAG_ENABLE，必须是组内第一条）
const AP_Param::GroupInfo FD_EKF::var_info[] = {
    // @Param: EN
    // @DisplayName: LOS delay-compensated EKF enable
    // @Description: 延迟补偿 EKF（7 状态联合估计：两通道 θ/θ̇/k + 共享延迟 Δ）开关。0=不运行（本组其余参数不在参数列表显示）；1=并行影子模式，EKF 与 LPF 链同时运行、UAT2 记录两者对照，但制导仍用 LPF 链；2=EKF 接管制导（LOS_RATE 阶段的 -PID 项改用 EKF 的 LOS rate，TRC 角度项与滞后偏置不变）。先在 1 上挂飞看 UAT2 差异，确认无误后再切 2。
    // @Values: 0:Disabled,1:ShadowOnly,2:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("EN", 1, FD_EKF, enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: Q
    // @DisplayName: LOS EKF process noise scale
    // @Description: 延迟补偿 EKF 的过程噪声标度。1.0 = 基准：未建模角加速度 σ≈2deg/s²（θ̇ 的随机游走），k=Ṙ/R 随机游走 σ≈0.05/s²。目标机动剧烈、滤波器跟不上（UAT3 的 InY/InP 归一化新息长期 >3）时调大；θ̇ 噪声偏大、抖动明显时调小。滤波器内部另有新息卡方检验，机动时自动临时放大 50 倍，此处只需给常态值。
    // @Range: 0.01 100
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("Q", 2, FD_EKF, q, 1.0f),

    // @Param: R
    // @DisplayName: LOS EKF measurement noise (deg)
    // @Description: 延迟补偿 EKF 的量测噪声标准差（deg）。量测是原始离轴角（去旋转在滤波器内部用状态 Δ̂ 完成）。按导引头像素量化+检测噪声折算：像素噪声 0.1° 量级起步取 0.15；若 UAT3 的 InY/InP 长期远小于 1（滤波器过度相信量测、θ̇ 抖），调大；若长期很大（跟不上量测），调小或调大 UATK_EKF_Q。
    // @Units: deg
    // @Range: 0.01 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("R", 3, FD_EKF, r, 0.15f),

    // @Param: QD
    // @DisplayName: LOS EKF delay-state process noise scale
    // @Description: EKF 里延迟状态 Δ（第 7 维，单位秒，随机游走）的过程噪声标度，乘在基准谱密度上。基准 1.0 对应 σ_Δ̇≈10ms/√s。Δ 跟踪慢、跟不上真实管线延迟漂移（UATK_DEBUG bit8 打印的 Dly ek 长期滞后于 lp）时调大；bit8 打印的 SgD（Δ 的 1σ）长期偏大、Δ̂ 抖时调小。默认 1.0（与原编译期常量一致，行为不变）。
    // @Range: 0.0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("QD", 4, FD_EKF, qd, 1.0f),

    // @Param: DMAX
    // @DisplayName: LOS EKF max delay state (ms)
    // @Description: EKF 延迟状态 Δ 的上限（ms），下限固定 10ms。相机管线延迟实测 40~60ms，默认 200ms 有充足余量；若已知管线延迟更小，调小可加快收敛并减少 Δ 与 LOS 运动之间的误分。最大可设 200ms（受状态历史缓冲容量约束）。
    // @Units: ms
    // @Range: 10 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("DMAX", 5, FD_EKF, dmax_ms, 200.0f),

    // @Param: SIG0
    // @DisplayName: LOS EKF initial delay uncertainty (ms)
    // @Description: EKF 首帧初始化时延迟状态 Δ 的 1σ 不确定度（ms）。默认 20ms（1σ 覆盖 40~60ms 的实测范围）。若 UATK_LPF_DLY 初值可信度高（已离线标定确认），调小到 5~10 可减少初期 Δ 游走；反之调大。
    // @Units: ms
    // @Range: 1 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SIG0", 6, FD_EKF, sig0_ms, 20.0f),

    AP_GROUPEND
};

FD_EKF::FD_EKF()
{
    AP_Param::setup_object_defaults(this, var_info);
    // Δ 先取 50ms 默认值；真实初值与 Q/R/QD/DMAX/SIG0 由 FD_Attack::init() 按参数重下
    // （构造时刻参数尚未加载）
    init(50.0f);
}

void FD_EKF::init(float d0_ms)
{
    _measurement_sigma_deg = 0.15f;
    _process_scale = 1.0f;

    for (uint8_t i = 0; i < N; i++) {
        _x[i] = 0.0f;
    }
    _x[I_DLY] = constrain_float(d0_ms, D_MIN_S * 1000.0f, _d_max_s * 1000.0f) * 0.001f;
    mat_identity(_P);
    _hist_next = 0;
    _last_ms = 0;
    _initialised = false;
    _meas_theta_deg[0] = _meas_theta_deg[1] = 0.0f;
    _innovation_deg[0] = _innovation_deg[1] = 0.0f;
    _innovation_norm[0] = _innovation_norm[1] = 0.0f;
    _innov_bad_count = 0;
    _reject_count = 0;
    _maneuver_until_ms = 0;
    _gyro_ema.zero();
    _hist_limited = false;

    for (uint8_t i = 0; i < HIST_SIZE; i++) {
        _hist[i].time_ms = 0;
        _hist[i].valid = false;
    }
}

float FD_EKF::process_scale_now(uint32_t now_ms) const
{
    // 机动兜底窗口内放大过程噪声，让滤波器"重新打开"，窗口过后自动回落
    if ((_maneuver_until_ms != 0) && (now_ms < _maneuver_until_ms)) {
        return _process_scale * MANEUVER_Q_BOOST;
    }
    return _process_scale;
}

// 过程模型单步传播。两个通道块各自走 θ̈=−2kθ̇、k̇=−k²；Δ 随机游走（F 为单位元）
void FD_EKF::predict_state(float (&x)[N], float P[N][N], float dt_s, float scale) const
{
    if (dt_s <= 0.0f) {
        return;
    }

    float remaining_s = dt_s;
    while (remaining_s > 0.0f) {
        const float dt = (remaining_s > MAX_STEP_S) ? MAX_STEP_S : remaining_s;
        remaining_s -= dt;

        for (uint8_t c = 0; c < 2; c++) {
            const uint8_t i_th = (c == 0) ? I_TH_Y : I_TH_P;
            const uint8_t i_td = (c == 0) ? I_TD_Y : I_TD_P;
            const uint8_t i_k  = (c == 0) ? I_K_Y  : I_K_P;
            const float k = x[i_k];
            const float td = x[i_td];
            x[i_th] += td * dt;
            x[i_td] += (-2.0f * k * td) * dt;
            x[i_k]  += (-k * k) * dt;
            x[i_k]  = constrain_float(x[i_k], -K_LIMIT, K_LIMIT);
            x[i_td] = constrain_float(x[i_td], -THETA_DOT_LIMIT_DPS, THETA_DOT_LIMIT_DPS);
        }
        // Δ：随机游走，状态不变

        // F = diag(F_yaw, F_pitch, 1)，每块 F = I + A·dt，A = [[0,1,0],[0,-2k,-2θ̇],[0,0,-2k]]
        float F[N][N];
        mat_identity(F);
        for (uint8_t c = 0; c < 2; c++) {
            const uint8_t i_th = (c == 0) ? I_TH_Y : I_TH_P;
            const uint8_t i_td = (c == 0) ? I_TD_Y : I_TD_P;
            const uint8_t i_k  = (c == 0) ? I_K_Y  : I_K_P;
            const float k = x[i_k];
            const float td = x[i_td];
            F[i_th][i_td] = dt;
            F[i_td][i_td] = 1.0f - 2.0f * k * dt;
            F[i_td][i_k]  = -2.0f * td * dt;
            F[i_k][i_k]   = 1.0f - 2.0f * k * dt;
        }

        float FP[N][N];
        float FPFt[N][N];
        mat_mul(F, P, FP);
        mat_mul_transpose_right(FP, F, FPFt);

        // Q·dt：θ/θ̇/k 吃机动放大量；Δ 只吃基准标度（目标机动≠延迟漂移）
        for (uint8_t c = 0; c < 2; c++) {
            const uint8_t i_th = (c == 0) ? I_TH_Y : I_TH_P;
            const uint8_t i_td = (c == 0) ? I_TD_Y : I_TD_P;
            const uint8_t i_k  = (c == 0) ? I_K_Y  : I_K_P;
            FPFt[i_th][i_th] += Q_THETA_BASE * dt * scale;
            FPFt[i_td][i_td] += Q_THETADOT_BASE * dt * scale;
            FPFt[i_k][i_k]   += Q_K_BASE * dt * scale;
        }
        // Δ 过程噪声 = 基准谱密度 × 总标度（UATK_EKF_Q）× Δ 专用标度（UATK_EKF_QD）
    FPFt[I_DLY][I_DLY] += Q_DLY_BASE * dt * _process_scale * _dly_q_scale;

        mat_copy(FPFt, P);

        // 对称化：抑制浮点漂移累积出的非对称分量
        for (uint8_t i = 0; i < N; i++) {
            for (uint8_t j = (uint8_t)(i + 1); j < N; j++) {
                const float s = 0.5f * (P[i][j] + P[j][i]);
                P[i][j] = s;
                P[j][i] = s;
            }
        }
    }
}

void FD_EKF::write_snapshot(uint32_t time_ms)
{
    Snapshot &s = _hist[_hist_next];
    s.time_ms = time_ms;
    for (uint8_t i = 0; i < N; i++) {
        s.x[i] = _x[i];
    }
    mat_copy(_P, s.P);
    s.valid = true;
    _hist_next = (uint8_t)((_hist_next + 1) % HIST_SIZE);
}

int8_t FD_EKF::find_snapshot_index(uint32_t t_ms) const
{
    int8_t best_idx = -1;
    uint32_t best_age_ms = 0;

    for (uint8_t i = 0; i < HIST_SIZE; i++) {
        const Snapshot &s = _hist[i];
        if (!s.valid || (s.time_ms > t_ms)) {
            continue;
        }
        const uint32_t age_ms = t_ms - s.time_ms;
        if ((best_idx < 0) || (age_ms < best_age_ms)) {
            best_idx = (int8_t)i;
            best_age_ms = age_ms;
        }
    }
    return best_idx;
}

void FD_EKF::propagate(uint32_t now_ms)
{
    if (!_initialised) {
        _last_ms = now_ms;
        return;
    }

    const uint32_t dt_ms = now_ms - _last_ms;
    if (dt_ms == 0) {
        return;
    }
    if (dt_ms > 2000U) {
        // 时钟跳变或长期未被调度：不做无意义的超长传播，直接对齐时间
        _last_ms = now_ms;
        return;
    }

    predict_state(_x, _P, (float)dt_ms * 0.001f, process_scale_now(now_ms));
    _last_ms = now_ms;
    write_snapshot(now_ms);
}

void FD_EKF::eval_h(float theta_y_deg, float theta_p_deg,
                    const Matrix3f &rotation_ned_from_camera, Vector2f &h_deg)
{
    // 由惯性系 LOS 方向（θy,θp）预测相机离轴角：e_ned → 相机系 → 方位/俯仰
    const float ty = radians(theta_y_deg);
    const float tp = radians(theta_p_deg);
    const Vector3f e(cosf(tp) * cosf(ty), cosf(tp) * sinf(ty), -sinf(tp));
    Matrix3f rotation_camera_from_ned = rotation_ned_from_camera;
    rotation_camera_from_ned.transpose();
    const Vector3f c = rotation_camera_from_ned * e;
    h_deg.x = degrees(atan2f(c.y, c.x));
    h_deg.y = degrees(atan2f(-c.z, c.xy().length()));
}

bool FD_EKF::derotate_meas(const Vector2f &target_c_deg, uint32_t t_att_ms,
                           const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body,
                           Vector2f &theta_deg)
{
    Vector3f att_deg;
    if (!attitude_delay.get_at_time_angle(t_att_ms, att_deg)) {
        return false;
    }
    const float yaw_c_rad = radians(target_c_deg.x);
    const float pitch_c_rad = radians(target_c_deg.y);
    const Vector3f target_c_unit(cosf(pitch_c_rad) * cosf(yaw_c_rad),
                                 cosf(pitch_c_rad) * sinf(yaw_c_rad),
                                 -sinf(pitch_c_rad));
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(att_deg.x), radians(att_deg.y), radians(att_deg.z));
    Vector3f e = (rotation_ned_from_body * rot_cam_to_body) * target_c_unit;
    e.normalize();
    theta_deg.x = wrap_180(degrees(atan2f(e.y, e.x)));
    theta_deg.y = degrees(atan2f(-e.z, e.xy().length()));
    return true;
}

void FD_EKF::push_observation(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                              const UDelay &attitude_delay, const Vector3f &gyro_b_dps,
                              const Matrix3f &rot_cam_to_body)
{
    if (!isfinite(target_c_deg.x) || !isfinite(target_c_deg.y)) {
        return;
    }

    // 单向机动识别（逐分量慢攻快放 EMA，按相机帧率）：瞬时值大于 EMA → 慢攻确认，
    // 防止零均值晃动误触发；回落 → 快放，机动结束后尽快恢复 Δ 通道更新
    for (uint8_t i = 0; i < 3; i++) {
        const float g = gyro_b_dps[i];
        const float a = (fabsf(g) > fabsf(_gyro_ema[i])) ? EMA_ATTACK : EMA_RELEASE;
        _gyro_ema[i] += a * (g - _gyro_ema[i]);
    }

    const uint32_t d_ms = (uint32_t)constrain_float(_x[I_DLY] * 1000.0f, 0.0f, 2000.0f);

    if (!_initialised) {
        // 首帧：θ 取本帧实测方向（状态 Δ̂ 去旋转），θ̇/k 置零，Δ 保持 init 初值，协方差放宽
        Vector2f th0;
        if (!derotate_meas(target_c_deg, t_arr_ms - d_ms, attitude_delay, rot_cam_to_body, th0) &&
            !derotate_meas(target_c_deg, t_arr_ms, attitude_delay, rot_cam_to_body, th0)) {
            return;   // 姿态缓冲尚未建立，下一帧再初始化
        }
        _x[I_TH_Y] = th0.x;
        _x[I_TD_Y] = 0.0f;
        _x[I_K_Y]  = 0.0f;
        _x[I_TH_P] = th0.y;
        _x[I_TD_P] = 0.0f;
        _x[I_K_P]  = 0.0f;
        mat_identity(_P);
        const float meas_var = _measurement_sigma_deg * _measurement_sigma_deg;
        _P[I_TH_Y][I_TH_Y] = meas_var;
        _P[I_TD_Y][I_TD_Y] = 900.0f;
        _P[I_K_Y][I_K_Y]   = 0.25f;
        _P[I_TH_P][I_TH_P] = meas_var;
        _P[I_TD_P][I_TD_P] = 900.0f;
        _P[I_K_P][I_K_P]   = 0.25f;
        _P[I_DLY][I_DLY]   = _d_init_sigma_s * _d_init_sigma_s;
        _meas_theta_deg[0] = th0.x;
        _meas_theta_deg[1] = th0.y;
        _innov_bad_count = 0;
        _reject_count = 0;
        _maneuver_until_ms = 0;
        for (uint8_t i = 0; i < HIST_SIZE; i++) {
            _hist[i].valid = false;
        }
        _hist_next = 0;
        _initialised = true;
        _last_ms = t_arr_ms - d_ms;
        write_snapshot(_last_ms);
        return;
    }

    kalman_update(target_c_deg, t_arr_ms, attitude_delay, rot_cam_to_body);
}

void FD_EKF::kalman_update(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                           const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body)
{
    // 量测真实时刻 = 到达时刻 − 状态 Δ̂；h 的姿态取数与之一致（同一 Δ̂）
    const uint32_t d_ms = (uint32_t)constrain_float(_x[I_DLY] * 1000.0f, 0.0f, 2000.0f);
    const uint32_t t_meas_ms = t_arr_ms - d_ms;

    // 快照回滚：找到 t_meas 处（或之前最近）的状态/协方差。
    // 回滚段长度受快照历史跨度限制（122 槽 @200Hz ≈610ms）：t_meas 早于最老快照时
    // 取不到快照、整帧跳过；加上 propagate() 的 dt>2000ms 早退与源切换必复位，
    // 正常运行时本段最多十几段 50ms 分段。
    // 边角：UATK_EKF_EN 由 0 拨回 1 的瞬间，上一段运行期的旧快照仍 valid，
    // 若此时 Δ̂ 毫秒内就来观测帧，可能选中几分钟前的旧快照做一次长传播——
    // 毫秒级一次性毛刺，P 长大会被后续钳位/重捕收住，不构成实际风险。
    const int8_t snap_idx = find_snapshot_index(t_meas_ms);
    if (snap_idx < 0) {
        _hist_limited = true;
        return;
    }
    _hist_limited = false;

    float x[N];
    float P[N][N];
    for (uint8_t i = 0; i < N; i++) {
        x[i] = _hist[snap_idx].x[i];
    }
    mat_copy(_hist[snap_idx].P, P);
    const uint32_t snap_t = _hist[snap_idx].time_ms;

    // 从快照前推到量测真实时刻（这段内无更新是正常的，协方差自然涨大）
    predict_state(x, P, (float)(t_meas_ms - snap_t) * 0.001f, process_scale_now(t_meas_ms));

    // h 需要的姿态：t_arr − Δ̂（与量测时刻口径一致）
    Vector3f att_deg;
    if (!attitude_delay.get_at_time_angle(t_arr_ms - d_ms, att_deg)) {
        return;   // 姿态缓冲未覆盖（启动早期），本帧放弃
    }
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(att_deg.x), radians(att_deg.y), radians(att_deg.z));
    const Matrix3f rotation_ned_from_camera = rotation_ned_from_body * rot_cam_to_body;

    // 本帧实测方向（几何量，ω 组装与偏航门控都用它）
    Vector2f th_meas;
    if (!derotate_meas(target_c_deg, t_arr_ms - d_ms, attitude_delay, rot_cam_to_body, th_meas)) {
        return;
    }

    // h(x) 与数值 H：∂h/∂θy、∂h/∂θp 扰动状态；∂h/∂Δ 扰动延迟重取姿态（= −ω_eff）
    Vector2f h0;
    eval_h(x[I_TH_Y], x[I_TH_P], rotation_ned_from_camera, h0);

    float H[2][N];
    for (uint8_t row = 0; row < 2; row++) {
        for (uint8_t j = 0; j < N; j++) {
            H[row][j] = 0.0f;
        }
    }
    {
        float xs[N];
        Vector2f h1;
        for (uint8_t i = 0; i < N; i++) { xs[i] = x[i]; }
        xs[I_TH_Y] += H_EPS_THETA_DEG;
        eval_h(xs[I_TH_Y], xs[I_TH_P], rotation_ned_from_camera, h1);
        H[0][I_TH_Y] = wrap_180(h1.x - h0.x) / H_EPS_THETA_DEG;
        H[1][I_TH_Y] = (h1.y - h0.y) / H_EPS_THETA_DEG;
        for (uint8_t i = 0; i < N; i++) { xs[i] = x[i]; }
        xs[I_TH_P] += H_EPS_THETA_DEG;
        eval_h(xs[I_TH_Y], xs[I_TH_P], rotation_ned_from_camera, h1);
        H[0][I_TH_P] = wrap_180(h1.x - h0.x) / H_EPS_THETA_DEG;
        H[1][I_TH_P] = (h1.y - h0.y) / H_EPS_THETA_DEG;
    }
    {
        Vector3f att2_deg;
        const uint32_t d2_ms = d_ms + (uint32_t)(H_EPS_DLY_S * 1000.0f);
        if (attitude_delay.get_at_time_angle(t_arr_ms - d2_ms, att2_deg)) {
            Matrix3f R2;
            R2.from_euler(radians(att2_deg.x), radians(att2_deg.y), radians(att2_deg.z));
            Vector2f h2;
            eval_h(x[I_TH_Y], x[I_TH_P], R2 * rot_cam_to_body, h2);
            H[0][I_DLY] = wrap_180(h2.x - h0.x) / H_EPS_DLY_S;
            H[1][I_DLY] = (h2.y - h0.y) / H_EPS_DLY_S;
        }
        // 取不到更老姿态时 Δ 列保持 0：本帧 Δ 不可观，只更新 θ（无害）
    }
    // Δ 列补上"视线自身运动"项。上面数值求得的只是姿态贡献；延迟改变时量测时刻
    // t_meas 随之移动，滤波器在该时刻的 θ 也沿 θ̇ 移动（∂θ/∂Δ = −θ̇）：
    //     ∂b/∂Δ|完整 = 姿态项 − (H_θy·θ̇y + H_θp·θ̇p)
    // 补上后即便机体不转（纯平飞横穿），Δ 也能从视线自身运动中被估出；单向机动
    // （直流视线角速度）下 Δ 与缓变 LOS 仍不可分——由冻结逻辑与 χ² 兜底负责。
    for (uint8_t row = 0; row < 2; row++) {
        H[row][I_DLY] -= H[row][I_TH_Y] * x[I_TD_Y] + H[row][I_TH_P] * x[I_TD_P];
    }

    // 偏航行门控：视线近天顶/天底时 ∂yaw_c/∂θy∝cosθp→0，增益本已收缩，
    // 但新息被姿态误差主导会让归一化新息虚高、误触机动兜底——整行剔除
    const bool use_yaw = fabsf(cosf(radians(th_meas.y))) > MIN_COS_THETA_PITCH;
    uint8_t rows[2];
    uint8_t m = 0;
    if (use_yaw) {
        rows[m++] = 0;
    }
    rows[m++] = 1;

    float innov[2];
    innov[0] = wrap_180(target_c_deg.x - h0.x);
    innov[1] = target_c_deg.y - h0.y;

    // 量测硬门限：任一有效行新息超限判离群点，本帧不更新；
    // 连续 REACQUIRE_COUNT 帧被拒 => 真实大跳变（源切换/重捕），θ 重置到实测方向并放大协方差
    bool outlier = false;
    for (uint8_t row = 0; row < m; row++) {
        if (fabsf(innov[rows[row]]) > INNOV_REJECT_DEG) {
            outlier = true;
        }
    }
    if (outlier) {
        if (_reject_count < 255) {
            _reject_count++;
        }
        if (_reject_count < REACQUIRE_COUNT) {
            _innovation_deg[0] = innov[0];
            _innovation_deg[1] = innov[1];
            _innovation_norm[0] = fabsf(innov[0]) / MAX(_measurement_sigma_deg, 0.01f);
            _innovation_norm[1] = fabsf(innov[1]) / MAX(_measurement_sigma_deg, 0.01f);
            return;
        }
        x[I_TH_Y] = th_meas.x;
        x[I_TH_P] = th_meas.y;
        for (uint8_t i = 0; i < N; i++) {
            P[i][i] *= REACQUIRE_P_SCALE;
        }
        _maneuver_until_ms = t_arr_ms + MANEUVER_HOLD_MS;
        // 本帧按【纯状态重置】处理，不做量测更新——否则紧接着的 Joseph 更新
        // (I−KH)P(I−KH)ᵀ + KRKᵀ 会把刚放大 100 倍的 P 当帧压回 ~R 量级，重置意图被自己抵消
        // （状态已强设为实测方向、新息置 0，继续做"量测更新"没有意义）。
        // 时间基用到达时刻 t_arr_ms：is_maneuver_boost()/process_scale_now() 拿当前
        // millis() 与 _maneuver_until_ms 比较，用 t_meas 会少保持一个 Δ̂。
        _innovation_deg[0] = 0.0f;
        _innovation_deg[1] = 0.0f;
        _innovation_norm[0] = 0.0f;
        _innovation_norm[1] = 0.0f;
        _meas_theta_deg[0] = th_meas.x;
        _meas_theta_deg[1] = th_meas.y;
        commit_state(snap_idx, t_meas_ms, x, P);
        return;
    }
    _reject_count = 0;

    // S = HPHᵀ + R（m×m，m≤2），闭式求逆
    const float r_var = _measurement_sigma_deg * _measurement_sigma_deg;
    float S[2][2];
    for (uint8_t a = 0; a < m; a++) {
        for (uint8_t b = 0; b < m; b++) {
            float s_ab = 0.0f;
            for (uint8_t i = 0; i < N; i++) {
                for (uint8_t j = 0; j < N; j++) {
                    s_ab += H[rows[a]][i] * P[i][j] * H[rows[b]][j];
                }
            }
            S[a][b] = s_ab + ((a == b) ? r_var : 0.0f);
        }
    }
    float S_inv[2][2];
    if (m == 1) {
        if (!isfinite(S[0][0]) || (S[0][0] <= 0.0f)) {
            return;
        }
        S_inv[0][0] = 1.0f / S[0][0];
    } else {
        const float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (!isfinite(det) || (fabsf(det) < 1e-12f)) {
            return;
        }
        S_inv[0][0] = S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] = S[0][0] / det;
    }

    // K = PHᵀS⁻¹（N×m）；机动时 Δ 行置零（增益调度：直流 ω 下 Δ 与缓变 LOS 不可分）
    const bool freeze_delta = _gyro_ema.length() > MANEUVER_DPS;
    float K[N][2];
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t a = 0; a < m; a++) {
            float k_ia = 0.0f;
            for (uint8_t j = 0; j < N; j++) {
                for (uint8_t b = 0; b < m; b++) {
                    k_ia += P[i][j] * H[rows[b]][j] * S_inv[b][a];
                }
            }
            K[i][a] = k_ia;
        }
        if (i == I_DLY && freeze_delta) {
            K[i][0] = 0.0f;
            K[i][1] = 0.0f;
        }
    }

    // 状态更新
    for (uint8_t i = 0; i < N; i++) {
        float dx = 0.0f;
        for (uint8_t a = 0; a < m; a++) {
            dx += K[i][a] * innov[rows[a]];
        }
        x[i] += dx;
    }
    x[I_K_Y] = constrain_float(x[I_K_Y], -K_LIMIT, K_LIMIT);
    x[I_K_P] = constrain_float(x[I_K_P], -K_LIMIT, K_LIMIT);
    x[I_TD_Y] = constrain_float(x[I_TD_Y], -THETA_DOT_LIMIT_DPS, THETA_DOT_LIMIT_DPS);
    x[I_TD_P] = constrain_float(x[I_TD_P], -THETA_DOT_LIMIT_DPS, THETA_DOT_LIMIT_DPS);
    // θ 是角度，必须反绕圈（偏航平面病态时 θ 会一路绕圈）
    x[I_TH_Y] = wrap_180(x[I_TH_Y]);
    x[I_DLY] = constrain_float(x[I_DLY], D_MIN_S, _d_max_s);

    // 日志量：分行新息与归一化新息（σ 数）。按有效行 rows[] 的映射写回——
    // 偏航行被门控剔除时（use_yaw=false）只有俯仰行参与更新（rows[0]=1、m=1，
    // S[0][0] 是俯仰行的 S），两列不能按固定行号对号入座。
    _innovation_deg[0] = innov[0];
    _innovation_deg[1] = innov[1];
    if (use_yaw) {
        _innovation_norm[0] = fabsf(innov[0]) / sqrtf(MAX(S[0][0], 1e-12f));
        _innovation_norm[1] = fabsf(innov[1]) / sqrtf(MAX(S[1][1], 1e-12f));
    } else {
        _innovation_norm[0] = 0.0f;   // 偏航行未参与更新，本帧无偏航归一化新息
        _innovation_norm[1] = fabsf(innov[1]) / sqrtf(MAX(S[0][0], 1e-12f));
    }
    _meas_theta_deg[0] = th_meas.x;
    _meas_theta_deg[1] = th_meas.y;

    // 机动兜底：χ² = innovᵀS⁻¹innov（只看有效行）
    float chi2 = 0.0f;
    for (uint8_t a = 0; a < m; a++) {
        for (uint8_t b = 0; b < m; b++) {
            chi2 += innov[rows[a]] * S_inv[a][b] * innov[rows[b]];
        }
    }
    if (chi2 > MANEUVER_CHI2) {
        if (_innov_bad_count < 255) {
            _innov_bad_count++;
        }
    } else {
        _innov_bad_count = 0;
    }
    if ((_innov_bad_count >= MANEUVER_TRIGGER_COUNT) || (chi2 > 100.0f)) {
        // 时间基用 t_arr_ms（同 reacquire 分支：用 t_meas 会少保持一个 Δ̂）
        _maneuver_until_ms = t_arr_ms + MANEUVER_HOLD_MS;
    }

    // Joseph 形式：P = (I−KH)P(I−KH)ᵀ + KRKᵀ
    float IKH[N][N];
    mat_identity(IKH);
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            float kh = 0.0f;
            for (uint8_t a = 0; a < m; a++) {
                kh += K[i][a] * H[rows[a]][j];
            }
            IKH[i][j] -= kh;
        }
    }
    float tmp[N][N];
    float P_new[N][N];
    mat_mul(IKH, P, tmp);
    mat_mul_transpose_right(tmp, IKH, P_new);
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            float krk = 0.0f;
            for (uint8_t a = 0; a < m; a++) {
                krk += K[i][a] * r_var * K[j][a];
            }
            P_new[i][j] += krk;
        }
    }
    mat_copy(P_new, P);

    commit_state(snap_idx, t_meas_ms, x, P);
}

// 把（位于 t_meas 的）状态/协方差沿快照时间轴推回最新时刻并覆盖沿途快照，
// 最后作为滤波器当前状态（= t_now）。量测更新路径与"重捕纯重置"路径共用。
// 注意 x/P 进入时位于 t_meas（不是 snap_t），dt 必须用独立时间游标 cur_t 起算；
// 若用相邻快照时间差，第一步会把 snap_t→t_meas 这段重复传播一遍。
void FD_EKF::commit_state(uint8_t snap_idx, uint32_t t_meas_ms, float (&x)[N], float P[N][N])
{
    uint8_t idx = snap_idx;
    uint32_t cur_t = t_meas_ms;
    for (uint8_t n = 0; n < HIST_SIZE; n++) {
        const uint8_t next = (uint8_t)((idx + 1) % HIST_SIZE);

        if (!_hist[next].valid) {
            break;
        }
        if (_hist[next].time_ms <= _hist[idx].time_ms) {
            // 已走到最新快照（再往前就回绕到最老样本）
            break;
        }

        const float dt_s = (float)(_hist[next].time_ms - cur_t) * 0.001f;
        predict_state(x, P, dt_s, process_scale_now(_hist[next].time_ms));
        cur_t = _hist[next].time_ms;
        for (uint8_t i = 0; i < N; i++) {
            _hist[next].x[i] = x[i];
        }
        mat_copy(P, _hist[next].P);
        idx = next;
    }

    // 滤波器当前状态 = 最新快照时刻的状态（即 t_now）
    for (uint8_t i = 0; i < N; i++) {
        _x[i] = x[i];
    }
    mat_copy(P, _P);
}

void FD_EKF::soften(float p_scale)
{
    const float scale = MAX(p_scale, 1.0f);
    if (!_initialised) {
        return;
    }

    for (uint8_t i = 0; i < N; i++) {
        _P[i][i] *= scale;
    }
    // 快照协方差必须同步放大：否则回滚会退回"未放大"的旧协方差，软重置对最近 Δ 毫秒无效
    for (uint8_t n = 0; n < HIST_SIZE; n++) {
        if (!_hist[n].valid) {
            continue;
        }
        for (uint8_t i = 0; i < N; i++) {
            _hist[n].P[i][i] *= scale;
        }
    }
    _innov_bad_count = 0;
}

float FD_EKF::get_theta_deg(Channel ch) const
{
    return (ch == Channel::YAW) ? _x[I_TH_Y] : _x[I_TH_P];
}

float FD_EKF::get_theta_dot_dps(Channel ch) const
{
    return (ch == Channel::YAW) ? _x[I_TD_Y] : _x[I_TD_P];
}

float FD_EKF::get_k(Channel ch) const
{
    return (ch == Channel::YAW) ? _x[I_K_Y] : _x[I_K_P];
}

float FD_EKF::get_sigma_theta_dot_dps(Channel ch) const
{
    const float p = (ch == Channel::YAW) ? _P[I_TD_Y][I_TD_Y] : _P[I_TD_P][I_TD_P];
    return (p > 0.0f) ? sqrtf(p) : 0.0f;
}

float FD_EKF::get_innovation_deg(Channel ch) const
{
    return _innovation_deg[(ch == Channel::YAW) ? 0 : 1];
}

float FD_EKF::get_innovation_norm(Channel ch) const
{
    return _innovation_norm[(ch == Channel::YAW) ? 0 : 1];
}

bool FD_EKF::is_initialised(Channel ch) const
{
    // 联合滤波器两通道同时初始化（首帧同时给两通道赋值）
    (void)ch;
    return _initialised;
}

bool FD_EKF::is_maneuver_boost(Channel ch) const
{
    (void)ch;
    return (_maneuver_until_ms != 0) && (AP_HAL::millis() < _maneuver_until_ms);
}

float FD_EKF::get_meas_theta_deg(Channel ch) const
{
    return _meas_theta_deg[(ch == Channel::YAW) ? 0 : 1];
}

void FD_EKF::set_process_scale(float scale)
{
    // 下限保护：0 会让协方差停止增长、滤波器僵死
    _process_scale = MAX(scale, 0.01f);
}

void FD_EKF::set_measurement_sigma_deg(float sigma_deg)
{
    _measurement_sigma_deg = constrain_float(sigma_deg, 0.01f, 10.0f);
}

void FD_EKF::mat_identity(float m[N][N])
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void FD_EKF::mat_copy(const float src[N][N], float dst[N][N])
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            dst[i][j] = src[i][j];
        }
    }
}

void FD_EKF::mat_mul(const float a[N][N], const float b[N][N], float out[N][N])
{
    float tmp[N][N];
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            float s = 0.0f;
            for (uint8_t k = 0; k < N; k++) {
                s += a[i][k] * b[k][j];
            }
            tmp[i][j] = s;
        }
    }
    mat_copy(tmp, out);
}

// out = a · bᵗ
void FD_EKF::mat_mul_transpose_right(const float a[N][N], const float b[N][N], float out[N][N])
{
    float tmp[N][N];
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            float s = 0.0f;
            for (uint8_t k = 0; k < N; k++) {
                s += a[i][k] * b[j][k];
            }
            tmp[i][j] = s;
        }
    }
    mat_copy(tmp, out);
}

void FD_EKF::set_dly_process_scale(float scale)
{
    _dly_q_scale = constrain_float(scale, 0.0f, 10.0f);
}

void FD_EKF::set_dly_max_ms(float ms)
{
    _d_max_s = constrain_float(ms * 0.001f, D_MIN_S, D_MAX_LIMIT_S);
}

void FD_EKF::set_dly_init_sigma_ms(float ms)
{
    _d_init_sigma_s = constrain_float(ms * 0.001f, 0.001f, D_MAX_LIMIT_S);
}
