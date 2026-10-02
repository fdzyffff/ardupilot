#include "FD_LPF.h"

// 参数组 UATK_LPF_*（EN 带 AP_PARAM_FLAG_ENABLE，必须是组内第一条）
const AP_Param::GroupInfo FD_LPF::var_info[] = {
    // @Param: EN
    // @DisplayName: LPF LOS chain enable
    // @Description: 启用“差分+低通”LOS 估计链。0=关闭（本组其余参数不在参数列表显示）；1=启用。关闭时若 EKF 未接管制导（UATK_EKF_EN<2），本链会被强制启用——LOS 必须有一条链在跑。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("EN", 1, FD_LPF, enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: HZ
    // @DisplayName: LOS rate low-pass filter frequency
    // @Description: “差分+低通”链里对惯性系 LOS 角速度的低通截止频率（Hz）。低通按本帧实测间隔施加，与调度/相机帧率无关，因此该值就是真实截止频率。调小更平滑但滞后更大；调大响应快但噪声大。
    // @Units: Hz
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HZ", 2, FD_LPF, hz, 2.0f),

    // @Param: DLY
    // @DisplayName: Camera pipeline delay (ms)
    // @Description: 相机管线延迟初值/固定值（ms）。0 等于关闭延迟补偿，而实测管线延迟 Δ≈40~60ms（2026-09-26 日志双通道互相关），填 0 会把残留错位 ω×Δ 在晃动时放大成虚假 LOS rate。取值链路按时间戳插值（UDelay::get_at_time_angle），本参数是“有效延迟”的初值/回退值；UATK_LPF_EST=1 时运行值由 UCamDelayEstimator 在线估计接管（估计器也以本参数为初值）。
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DLY", 3, FD_LPF, dly_ms, 50),

    // @Param: EST
    // @DisplayName: Camera delay online estimator enable
    // @Description: 1=CAM 源时启用 UCamDelayEstimator 在线估计相机管线延迟 Δ 并接管有效延迟 _dly_lpf（原始离轴角+姿态历史相位对齐，缓冲原始 b 重放姿态，窗口去趋势回归，机动/弱激励门控冻结）；0=固定使用 UATK_LPF_DLY。晃动工况下延迟误差 ω×Δ_err 会漏成虚假 LOS rate，在线估计可跟踪 Δ 漂移；不晃时 Δ 不可观测也无危害，估计器冻结保持最后值。估计器初值取 UATK_LPF_DLY（内部钳位 [10,200]ms）。注意：本开关只影响 LPF 链与延迟姿态取数；EKF 链的 Δ 是滤波器状态，不与其联动。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("EST", 4, FD_LPF, est_en, 0),

    AP_GROUPEND
};

FD_LPF::FD_LPF()
{
    AP_Param::setup_object_defaults(this, var_info);
    // 截止频率随后由 FD_Attack::init() 按参数下发
    init();
}

void FD_LPF::init()
{
    _los_e_unit.zero();
    _los_e_deg.zero();
    _los_e_dot.zero();
    _los_e_rate_dps.zero();
    _los_c_rate_dps.zero();
    _initialised = false;
    reset();
}

void FD_LPF::reset()
{
    _filter_x.reset();
    _filter_y.reset();
    _filter_z.reset();
    _derivative_x.reset();
    _derivative_y.reset();
    _derivative_z.reset();
    _initialised = false;
}

void FD_LPF::set_cutoff_hz(float cutoff_hz)
{
    // 变 dt 版本：只需给截止频率，dt 由 update() 每次传入
    _filter_x.set_cutoff_frequency(cutoff_hz);
    _filter_y.set_cutoff_frequency(cutoff_hz);
    _filter_z.set_cutoff_frequency(cutoff_hz);
}

void FD_LPF::update(const Vector3f &los_c_unit, const Matrix3f &rotation_ned_from_camera,
                    uint32_t now_ms, float dt_s)
{
    // 1) 去旋转到惯性系 NED 并归一化
    _los_e_unit = rotation_ned_from_camera * los_c_unit;
    _los_e_unit.normalize();

    // 2) 由单位向量直接得 θ
    _los_e_deg.x = wrap_180(degrees(atan2f(_los_e_unit.y, _los_e_unit.x)));
    _los_e_deg.y = degrees(atan2f(-_los_e_unit.z, _los_e_unit.xy().length()));
    _initialised = true;

    // 3) 三个分量各自数值求导，再减去沿 LOS 的径向分量
    _derivative_x.update(_los_e_unit.x, now_ms);
    _derivative_y.update(_los_e_unit.y, now_ms);
    _derivative_z.update(_los_e_unit.z, now_ms);

    _los_e_dot.x = _derivative_x.slope() * 1000.0f;
    _los_e_dot.y = _derivative_y.slope() * 1000.0f;
    _los_e_dot.z = _derivative_z.slope() * 1000.0f;
    _los_e_dot -= _los_e_unit * (_los_e_unit * _los_e_dot);

    // 4) ω_e = l_e × ḋl_e，低通放在求导之后（dt_s ≤ 0 时低通不推进）
    const Vector3f los_e_rate_rads = _los_e_unit % _los_e_dot;
    _los_e_rate_dps = Vector3f(_filter_x.apply(degrees(los_e_rate_rads.x), dt_s),
                               _filter_y.apply(degrees(los_e_rate_rads.y), dt_s),
                               _filter_z.apply(degrees(los_e_rate_rads.z), dt_s));

    // 5) 相机系 LOS rate
    Matrix3f rotation_camera_from_ned = rotation_ned_from_camera;
    rotation_camera_from_ned.transpose();
    _los_c_rate_dps = rotation_camera_from_ned * _los_e_rate_dps;
}

float FD_LPF::get_theta_deg(Channel ch) const
{
    return ch == Channel::YAW ? _los_e_deg.x : _los_e_deg.y;
}

// 球面基投影：θ̇p = ω·e_el，θ̇y = (ω·ê_az)/cosθp（与 FD_EKF 组装 ω 用的基向量严格互逆）
float FD_LPF::get_theta_dot_dps(Channel ch) const
{
    const float theta_y_rad = radians(_los_e_deg.x);
    const float theta_p_rad = radians(_los_e_deg.y);
    const float sin_ty = sinf(theta_y_rad);
    const float cos_ty = cosf(theta_y_rad);
    const float sin_tp = sinf(theta_p_rad);
    const float cos_tp = cosf(theta_p_rad);

    if (ch == Channel::PITCH) {
        // e_el = (−sinθy, cosθy, 0)
        return -_los_e_rate_dps.x * sin_ty + _los_e_rate_dps.y * cos_ty;
    }

    // ê_az = (sinθp·cosθy, sinθp·sinθy, cosθp)；近天顶/天底时方位角速率不可观测，记 0
    if (fabsf(cos_tp) <= MIN_COS_THETA_PITCH) {
        return 0.0f;
    }
    return (_los_e_rate_dps.x * sin_tp * cos_ty +
            _los_e_rate_dps.y * sin_tp * sin_ty +
            _los_e_rate_dps.z * cos_tp) / cos_tp;
}
