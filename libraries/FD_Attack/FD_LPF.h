#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <Filter/Filter.h>

/*
 * FD_LPF —— "差分+低通"LOS 估计链（参数组 UATK_LPF_* 与实现同一个实例）。
 *
 * 思路：相机离轴角用延迟后的姿态去旋转到惯性系，对 LOS 单位向量逐分量求导，
 * ω = l × ḋl，最后低通（低通放在求导【之后】，噪声不被放大）。所有 dt 按
 * 帧间隔实测，与调度/相机帧率无关。θ̇ 输出用球面基投影，与 FD_EKF 同口径；
 * 视线近天顶/天底时方位角速率不可观测，该帧记 0。
 *
 * 参数组首条 UATK_LPF_EN 带 AP_PARAM_FLAG_ENABLE：为 0 时组内其余参数不在
 * 参数列表显示（值仍加载保存）；实际链路开关在 FD_Attack::lpf_enabled()。
 */
class FD_LPF
{
public:
    // 两条链（FD_LPF / FD_EKF）各自维护同名通道枚举，输出接口逐列对齐
    enum class Channel : uint8_t {
        YAW = 0,
        PITCH = 1,
        NUM_CHANNELS = 2
    };

    FD_LPF();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    // 源切换 / 重新捕获：清滤波器状态，下一次 update() 用首帧样本自动初始化
    void reset();

    // 低通截止频率（Hz），由上层按 UATK_LPF_HZ 传入（1Hz 刷新一次即可）
    void set_cutoff_hz(float cutoff_hz);

    // 每帧相机量测调用一次。
    //   los_c_unit               相机系单位向量（由离轴角构造）
    //   rotation_ned_from_camera 延迟后的机体姿态 × 安装矩阵（R_ned_body·R_body_cam）
    //   now_ms                   本帧到达时刻（导数滤波器按它算 dt）
    //   dt_s                     本帧与上一帧的间隔（低通按它施加）
    void update(const Vector3f &los_c_unit, const Matrix3f &rotation_ned_from_camera,
                uint32_t now_ms, float dt_s);

    // ---- 与 FD_EKF 对齐的输出接口 ----
    float get_theta_deg(Channel ch) const;
    float get_theta_dot_dps(Channel ch) const;

    // ---- 原始量（供上层镜像到自己的成员，控制律/日志/打印统一取用）----
    const Vector3f &get_los_e_unit() const { return _los_e_unit; }
    const Vector2f &get_los_e_deg() const { return _los_e_deg; }
    const Vector3f &get_los_e_dot() const { return _los_e_dot; }
    const Vector3f &get_los_e_rate_dps() const { return _los_e_rate_dps; }
    const Vector3f &get_los_c_rate_dps() const { return _los_c_rate_dps; }
    bool is_initialised() const { return _initialised; }

    // ---- 参数（组名 LPF_，全名 UATK_LPF_*）----
    AP_Int8  enable;   // EN：1=启用本链；=0 且 EKF 未接管时会被强制启用（两条链不能都不跑）
    AP_Float hz;       // HZ：LOS 角速度低通截止频率（Hz）
    AP_Int16 dly_ms;   // DLY：相机管线延迟（ms）初值/固定值
    AP_Int8  est_en;   // EST：1=CAM 源时由 UCamDelayEstimator 在线估计 Δ 并接管

private:
    // 视线近天顶/天底时方位角速率不可观测的门限（与 FD_EKF 同值）
    static constexpr float MIN_COS_THETA_PITCH = 0.2f;

    Vector3f _los_e_unit;        // 惯性系 LOS 单位向量
    Vector2f _los_e_deg;         // 惯性系 LOS 角（度）：x=方位，y=俯仰
    Vector3f _los_e_dot;         // 单位向量导数去掉径向分量后（1/s）
    Vector3f _los_e_rate_dps;    // 惯性系 LOS 角速度（deg/s）
    Vector3f _los_c_rate_dps;    // 相机系 LOS 角速度（deg/s）
    bool _initialised;

    // 先对单位向量分量求导，再对所得角速度低通（低通在变 dt 模式下工作）
    DerivativeFilterFloat_Size7 _derivative_x;
    DerivativeFilterFloat_Size7 _derivative_y;
    DerivativeFilterFloat_Size7 _derivative_z;
    LowPassFilterFloat _filter_x;
    LowPassFilterFloat _filter_y;
    LowPassFilterFloat _filter_z;
};
