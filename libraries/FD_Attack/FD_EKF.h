#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include "UDelay.h"

/*
 * FD_EKF —— 延迟补偿 LOS 估计器（7 状态联合 EKF，参数组 UATK_EKF_* 与实现同一个实例）。
 *
 * 核心思想：相机管线延迟 Δ 不作外部参数，而是滤波器第 7 维状态，与偏航/俯仰
 * 两通道联合估计：x = [θy, θ̇y, ky, θp, θ̇p, kp, Δ]ᵀ，其中 k ≜ Ṙ/R。
 * 量测是【原始相机离轴角】（未去旋转），去旋转在量测函数内部用状态 Δ̂ 完成——
 * 这样 ∂h/∂Δ≠0，Δ 才可观。量测按真实时刻 t_arr−Δ̂ 做快照回滚更新，再重传播
 * 回当前时刻。不晃动时 Δ 增益内生归零；单向机动下 Δ 与缓变 LOS 不可分，由
 * 陀螺门控冻结 Δ 通道。参数只是初值/标度，运行值每周期下发，改参即时生效。
 */
class FD_EKF
{
public:
    // 两条链（FD_LPF / FD_EKF）各自维护同名通道枚举，输出接口逐列对齐
    enum class Channel : uint8_t {
        YAW = 0,
        PITCH = 1,
        NUM_CHANNELS = 2
    };

    FD_EKF();

    static const struct AP_Param::GroupInfo var_info[];

    // d0_ms：Δ 状态初值（取 UATK_LPF_DLY）
    void init(float d0_ms = 50.0f);

    // 每周期调用：按实际 dt 前推一步并写快照（每周期都跑，不等相机帧）
    void propagate(uint32_t now_ms);

    // 相机帧量测（原始离轴角 b，deg；t_arr_ms 为帧到达时刻）。
    // 去旋转在量测函数内部用状态 Δ̂ 完成；gyro_b_dps 仅用于机动冻结增益调度。
    void push_observation(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                          const UDelay &attitude_delay, const Vector3f &gyro_b_dps,
                          const Matrix3f &rot_cam_to_body);

    // 软重置：保留状态、放大协方差（乘性累积，无上限——只能低频偶发调用，不能放每周期路径上）
    void soften(float p_scale);

    float get_theta_deg(Channel ch) const;
    float get_theta_dot_dps(Channel ch) const;
    float get_k(Channel ch) const;
    // θ̇ 估计的 1σ（deg/s），供日志判读与末段降增益参考
    float get_sigma_theta_dot_dps(Channel ch) const;
    float get_innovation_deg(Channel ch) const;
    // 归一化新息（σ 数），>3 说明模型/量测不一致
    float get_innovation_norm(Channel ch) const;
    bool is_initialised(Channel ch) const;
    // 机动兜底是否处于激活状态（供日志判读）
    bool is_maneuver_boost(Channel ch) const;
    // 快照历史跨度不够（本帧回滚时刻早于最老快照，该帧跳过更新）——判读用
    bool is_history_limited() const { return _hist_limited; }

    // Δ 状态当前值（ms）——EKF 内部延迟估计，与 UCamDelayEstimator 相互独立
    float get_delay_ms() const { return _x[I_DLY] * 1000.0f; }
    // Δ 状态的 1σ（ms）
    float get_sigma_delay_ms() const { return sqrtf(MAX(_P[I_DLY][I_DLY], 0.0f)) * 1000.0f; }
    // 本帧实测 LOS 方向（用状态 Δ̂ 去旋转后的惯性系角，deg）：ω 组装的几何量取它——
    // 方向本身处处良态，速率量才取滤波器状态
    float get_meas_theta_deg(Channel ch) const;

    void set_process_scale(float scale);
    void set_measurement_sigma_deg(float sigma_deg);
    // Δ 状态的三项现场可调量（默认值 = 原编译期常量，默认行为不变；每周期刷新）
    void set_dly_process_scale(float scale);   // Δ 随机游走标度（× Q_DLY_BASE）
    void set_dly_max_ms(float ms);             // Δ 上限（钳位在 [D_MIN_S, D_MAX_LIMIT_S]）
    void set_dly_init_sigma_ms(float ms);      // Δ 初值不确定度 1σ（首帧协方差用）

    // ---- 参数（组名 EKF_，全名 UATK_EKF_*）----
    // EN 带 AP_PARAM_FLAG_ENABLE，必须是组内第一条（=0 时组内其余参数不显示）。
    // 三态语义：0=不运行；1=并行影子（只算只记，制导仍用 LPF 链）；2=EKF 接管制导
    AP_Int8  enable;
    AP_Float q;         // Q：过程噪声标度
    AP_Float r;         // R：量测噪声 std（deg）
    AP_Float qd;        // QD：Δ 状态过程噪声标度
    AP_Float dmax_ms;   // DMAX：Δ 状态上限（ms）
    AP_Float sig0_ms;   // SIG0：Δ 初值 1σ（ms）

private:
    // 状态维度与下标
    static constexpr uint8_t N = 7;
    static constexpr uint8_t I_TH_Y = 0;
    static constexpr uint8_t I_TD_Y = 1;
    static constexpr uint8_t I_K_Y  = 2;
    static constexpr uint8_t I_TH_P = 3;
    static constexpr uint8_t I_TD_P = 4;
    static constexpr uint8_t I_K_P  = 5;
    static constexpr uint8_t I_DLY  = 6;

    // Δ 状态钳位（s）与初值不确定度
    static constexpr float D_MIN_S = 0.010f;          // Δ 下限（固定，10ms）
    static constexpr float D_MAX_LIMIT_S = 0.200f;    // Δ 硬上限：UATK_EKF_DMAX 的钳位上限，也是 HIST_SIZE 的取槽依据
    static constexpr float D_INIT_SIGMA_S = 0.020f;   // Δ 初值 1σ 默认值（可被 UATK_EKF_SIG0 覆盖）

    // ---- 快照历史：按【时间跨度】取槽数，不写死帧数（调度频率变化时自适应）----
    static constexpr float HIST_RATE_HZ = 400.0f;
    static constexpr uint8_t HIST_SIZE = (uint8_t)(D_MAX_LIMIT_S * HIST_RATE_HZ * 1.5f) + 2;
    // 机动兜底：归一化新息平方超限连续计数达到该值即触发
    static constexpr uint8_t MANEUVER_TRIGGER_COUNT = 2;
    static constexpr float MANEUVER_CHI2 = 9.0f;      // 3σ（按行判定）
    static constexpr float MANEUVER_Q_BOOST = 50.0f;  // 触发时过程噪声放大量（只放大 θ/θ̇/k，不放大 Δ）
    static constexpr uint32_t MANEUVER_HOLD_MS = 500;
    static constexpr float MAX_STEP_S = 0.05f;        // 单步最大传播间隔，超长分段
    static constexpr float K_LIMIT = 10.0f;
    static constexpr float THETA_DOT_LIMIT_DPS = 1000.0f;
    // 量测硬门限：单帧新息绝对值超过此值判为离群点，本帧不更新。连续 REACQUIRE_COUNT 次被拒
    // 视为真实大跳变（源切换/重新捕获），把 θ 直接重置到实测方向并放大协方差
    static constexpr float INNOV_REJECT_DEG = 20.0f;
    static constexpr uint8_t REACQUIRE_COUNT = 5;
    static constexpr float REACQUIRE_P_SCALE = 100.0f;
    // 视线近天顶/天底时偏航行量测退化的门限（与 FD_LPF 同值）
    static constexpr float MIN_COS_THETA_PITCH = 0.2f;

    // 过程噪声基准谱密度（乘 _process_scale；机动放大量只作用在 θ/θ̇/k 上）
    static constexpr float Q_THETA_BASE = 1.0e-4f;    // θ：极小，θ 由 θ̇ 驱动
    static constexpr float Q_THETADOT_BASE = 4.0f;    // θ̇：未建模角加速度 σ_a ≈ 2 deg/s²
    static constexpr float Q_K_BASE = 2.5e-3f;        // k：随机游走 σ ≈ 0.05 (1/s)/s
    static constexpr float Q_DLY_BASE = 1.0e-4f;      // Δ：随机游走 σ ≈ 10ms/√s（机动放大量不作用于它）

    // 单向机动冻结 Δ 通道：陀螺 EMA（慢攻/快放，按相机帧率）范数门限
    static constexpr float MANEUVER_DPS = 10.0f;
    static constexpr float EMA_ATTACK  = 0.02f;
    static constexpr float EMA_RELEASE = 0.15f;

    // 数值 H 的扰动步长
    static constexpr float H_EPS_THETA_DEG = 0.02f;
    static constexpr float H_EPS_DLY_S = 0.002f;

    struct Snapshot {
        uint32_t time_ms;
        float x[N];
        float P[N][N];
        bool valid;
    };

    float _x[N];
    float _P[N][N];
    Snapshot _hist[HIST_SIZE];
    uint8_t _hist_next;
    uint32_t _last_ms;          // 最近一次传播到的时刻（= 最新快照时刻）
    bool _initialised;
    float _meas_theta_deg[2];   // 本帧实测方向（状态 Δ̂ 去旋转）
    float _innovation_deg[2];
    float _innovation_norm[2];
    uint8_t _innov_bad_count;
    uint8_t _reject_count;      // 连续被硬门限拒绝的帧数（用于自愈重捕）
    bool _hist_limited;         // 本帧因历史跨度不够被跳过
    uint32_t _maneuver_until_ms;
    Vector3f _gyro_ema;         // 单向机动识别（慢攻快放）

    float _measurement_sigma_deg;
    float _dly_q_scale = 1.0f;              // Δ 过程噪声标度（UATK_EKF_QD）
    float _d_max_s = D_MAX_LIMIT_S;         // Δ 上限（UATK_EKF_DMAX）
    float _d_init_sigma_s = D_INIT_SIGMA_S; // Δ 初值 1σ（UATK_EKF_SIG0）
    float _process_scale;

    float process_scale_now(uint32_t now_ms) const;

    // 过程模型单步传播（就地作用，便于快照回滚时在局部副本上使用）
    void predict_state(float (&x)[N], float P[N][N], float dt_s, float scale) const;
    void write_snapshot(uint32_t time_ms);
    // 把位于 t_meas 的状态/协方差推回最新时刻并覆盖沿途快照（量测更新与重捕重置共用）
    void commit_state(uint8_t snap_idx, uint32_t t_meas_ms, float (&x)[N], float P[N][N]);
    // 返回 hist 中时刻 <= t_ms 且最接近的快照下标；找不到返回 -1
    int8_t find_snapshot_index(uint32_t t_ms) const;
    void kalman_update(const Vector2f &target_c_deg, uint32_t t_arr_ms,
                       const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body);

    // 量测函数：由状态（θy,θp）与 t_att_ms 时刻姿态预测相机离轴角（deg）。
    // R_ned_cam 由调用方备好（避免重复查表）
    static void eval_h(float theta_y_deg, float theta_p_deg,
                       const Matrix3f &rotation_ned_from_camera, Vector2f &h_deg);
    // 由本帧实测方向给出去旋转后的惯性系角（用状态 Δ̂ 对应的姿态），供 ω 组装几何量
    static bool derotate_meas(const Vector2f &target_c_deg, uint32_t t_att_ms,
                              const UDelay &attitude_delay, const Matrix3f &rot_cam_to_body,
                              Vector2f &theta_deg);

    // 7x7 定点矩阵运算（避免依赖通用矩阵库）
    static void mat_identity(float m[N][N]);
    static void mat_copy(const float src[N][N], float dst[N][N]);
    static void mat_mul(const float a[N][N], const float b[N][N], float out[N][N]);
    static void mat_mul_transpose_right(const float a[N][N], const float b[N][N], float out[N][N]);
};
