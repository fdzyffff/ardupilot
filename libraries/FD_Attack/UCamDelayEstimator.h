#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "UDelay.h"

/*
 * UCamDelayEstimator —— 相机管线延迟 Δ 在线估计器（原始信号相位对齐方案），
 * 只服务 FD_LPF 链；FD_EKF 链的 Δ 在滤波器状态里，与本估计器无关。
 *
 * 思路：LOS 缓变先验下，离轴角的高频振荡几乎全由机体转动贡献。用当前 Δ̂ 去
 * 旋转窗口内各帧，残留振荡 ≈ ω·(Δ−Δ̂)，对 ω 的交流成分做最小二乘即得 Δ 修正量。
 * 缓冲【原始离轴角】、每次用当前 Δ̂ 重放姿态——缓冲合成角会混入旧 Δ̂ 形成
 * 正反馈。加固：窗口 yaw 解缠、机动冻结并回滚到干净窗确认值、Δ̂ 速率限幅。
 * 不晃动时 Δ 不可观测也不产生危害，激励不足即冻结保持最后值。
 */
class UCamDelayEstimator {
public:
    void init(float d0_ms);

    // 每个相机帧调用一次（仅 CAM 源、UATK_LPF_EST=1 时由 FD_Attack::update_lpf 调用）。
    //   target_c_deg   : 原始离轴角（deg，x=偏航/y=俯仰），未做任何去旋转
    //   t_arr_ms       : 帧到达时刻（AP_HAL::millis）
    //   attitude_delay : 姿态历史缓冲（角度走 ±180° 最短路径插值）
    //   gyro_b_dps     : 本周期机体陀螺（deg/s），仅用于单向机动门控
    //   rot_cam_to_body: 相机安装矩阵
    void update(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                const UDelay &attitude_delay, const Vector3f &gyro_b_dps,
                const Matrix3f &rot_cam_to_body);

    // 当前延迟估计值（ms）。冻结期返回最后估计值；从未更新过时返回初值（UATK_LPF_DLY）
    float get_delay_ms() const { return _d_s * 1000.0f; }

    // 本窗回归残差 RMS(deg)：扣掉去趋势项与 δ̂·ω 项后量测上还剩的逐帧角度散度
    // （残余逐帧延迟抖动 ω_eff·ε + 相机噪声）。激励不足的窗口不刷新，返回 0 表示"不可测"。
    float get_meas_scatter_deg() const { return _scatter_deg; }

private:
    static constexpr uint8_t  WIN = 30;               // 窗口帧数（≈1.0s@30Hz），受 UDelay 跨度约束
    static constexpr uint8_t  MIN_FRAMES = 15;        // 有效帧不足时不更新
    static constexpr uint16_t TRANSIENT_FRAMES = 15;  // 启动暂态不参与（缓冲未稳）
    static constexpr float    GAIN = 0.5f;            // 每相机帧更新增益，收敛时间常数 ≈0.3~0.5s（激励充足时）
    static constexpr float    DEADBAND_S = 0.0015f;   // 死区：小于 1.5ms 的修正不动
    static constexpr float    D_MIN_S = 0.010f;       // Δ̂ 钳位下限
    static constexpr float    D_MAX_S = 0.200f;       // Δ̂ 钳位上限
    static constexpr float    EXCITE_DPS = 15.0f;     // 激励门限：窗口内 ω 交流 RMS 低于此值不更新
    static constexpr float    MANEUVER_DPS = 10.0f;   // 机动门限：陀螺 EMA 范数超过即清空窗口并冻结
    static constexpr float    EMA_ATTACK  = 0.02f;    // 慢攻（τ≈1.7s@30Hz）：1.5Hz 零均值晃动通不过
    static constexpr float    EMA_RELEASE = 0.15f;    // 快放：机动结束后约 0.2s 恢复估计
    static constexpr float    PERTURB_S   = 0.005f;   // 数值灵敏度 ∂z/∂Δ̂ 的扰动步长 5ms
    static constexpr float    D_RATE_MAX  = 0.040f;   // Δ̂ 变化率上限 40ms/s
    static constexpr uint8_t  COMMIT_FRAMES = 30;     // 连续一整窗干净帧后把 Δ̂ 提交为冻结回滚基准

    struct RawFrame {
        Vector2f target_c_deg;
        uint32_t t_arr_ms;
    };
    RawFrame _buf[WIN];
    uint8_t  _count = 0;
    uint8_t  _head = 0;         // 环形缓冲下一写入位置
    float    _d_s = 0.05f;
    float    _d_commit = 0.05f; // 上一个"连续干净窗口"确认的 Δ̂——机动冻结时回滚到它
    uint8_t  _clean_frames = 0; // 距上次机动门控打断的连续正常帧数
    float    _scatter_deg = 0.0f;
    uint32_t _last_t_ms = 0;    // 上一帧到达时刻，用于 Δ̂ 速率限幅折算
    Vector3f _gyro_ema;
    uint32_t _n_seen = 0;

    // 用 t_att_ms 时刻的姿态把原始离轴角合成惯性系 LOS 角（deg）；
    // 姿态缓冲未覆盖该时刻返回 false，调用侧跳过该帧
    bool synth_los_deg(const Vector2f &target_c_deg, uint32_t t_att_ms,
                       const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body,
                       Vector2f &z_deg) const;

    // 窗口内最小二乘直线去趋势（LOS 缓变趋势一个窗口可达数度，大于延迟残差信号本身，
    // 不去趋势会把回归拖走；不能只去均值）
    static void detrend(const float *in, float *out, uint8_t n);
};
