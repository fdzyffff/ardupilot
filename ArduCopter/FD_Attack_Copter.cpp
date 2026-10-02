// FD_Attack_Copter —— Copter 端派生类实现：只做"控制量合成"。
// LOS/目标/估计/日志主体在基类 FD_Attack（libraries/FD_Attack/FD_Attack.cpp）；
// 本文件只填基类切好的"缝"：update_control 纯虚 + 五个钩子虚函数。
// 设计思路见 FD_Attack_Copter.h 文件头。
#include "FD_Attack_Copter.h"
#include "Copter.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// 参数序号约定：基类 FD_Attack 用 0~20（8~20 留空备用），本类（端侧派生）21 起。
// 参数全名前缀 UATK_ 由 ArduCopter/Parameters.cpp 的 GOBJECT(uattack, "UATK_", FD_Attack_Copter) 给出。
const AP_Param::GroupInfo FD_Attack_Copter::var_info[] = {
    // 基类（FD_Attack）的参数表：空名嵌套 → 参数全名仍是 UATK_*
    AP_NESTEDGROUPINFO(FD_Attack, 0),

    // ---- 本类（控制侧）参数 ----
    AP_GROUPINFO("YAW_P", 21, FD_Attack_Copter, yaw_angle_gain, 1.0f),
    AP_GROUPINFO("PIT_P", 22, FD_Attack_Copter, pitch_angle_gain, 1.0f),
    AP_GROUPINFO("RATE_MAX", 23, FD_Attack_Copter, rate_limit_dps, 30.0f),
    AP_GROUPINFO("VIS_LIM", 24, FD_Attack_Copter, visibility_limit_deg, 20.0f),
    AP_SUBGROUPINFO(los_yaw_rate_pid, "NY_", 25, FD_Attack_Copter, AC_PID),
    AP_SUBGROUPINFO(los_pitch_rate_pid, "NP_", 26, FD_Attack_Copter, AC_PID),

    // @Param: THR
    // @DisplayName: Attack throttle
    // @Description: Normalized throttle target used in Attack mode
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("THR", 27, FD_Attack_Copter, throttle, 0.5f),

    // @Param: THR_RATE
    // @DisplayName: Attack throttle change rate
    // @Description: Maximum normalized throttle change per second in Attack mode
    // @Units: 1/s
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("THR_RATE", 28, FD_Attack_Copter, throttle_rate, 0.1f),

    // @Param: RLL_P
    // @DisplayName: Camera earth-level roll gain
    // @Description: Camera roll rate gain used while camera pitch is below 80 degrees
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("RLL_P", 29, FD_Attack_Copter, roll_level_gain, 1.0f),

    // @Param: FWD_PIT
    // @DisplayName: Stage 3 forward pitch angle
    // @Description: Camera earth-frame pitch target used to add forward acceleration in Stage 3
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FWD_PIT", 30, FD_Attack_Copter, forward_pitch_deg, 30.0f),

    // @Param: TRC_Y
    // @DisplayName: Attack track yaw gain
    // @Description: Proportional pursuit gain on camera yaw angle error, added to LOS-rate PID output in Stage 3
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRC_Y", 31, FD_Attack_Copter, track_yaw_gain, 1.0f),

    // @Param: TRC_P
    // @DisplayName: Attack track pitch gain
    // @Description: Proportional pursuit gain on camera pitch angle error, added to LOS-rate PID output in Stage 3
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRC_P", 32, FD_Attack_Copter, track_pitch_gain, 1.0f),

    // @Param: FWD_EN
    // @DisplayName: Attack forward pitch rate enable
    // @Description: When set to 0 the camera pitch forward rate term in Stage 3 is forced to 0 (disabled); when non-zero the term is enabled。该偏置仅在无速度补偿时生效：UATK_VEL_USE=1 且速度有效时 FWD_PIT 被强制置 0。
    // @User: Advanced
    AP_GROUPINFO("FWD_EN", 33, FD_Attack_Copter, forward_pitch_en, 0),

    // @Param: LAG_P_HL
    // @DisplayName: Pitch lag compensation half-life
    // @Description: Half-life in seconds of the leaky integrator estimating how far the velocity direction lags the camera pitch axis (sideslip). 0 disables pitch compensation. Effective time constant is half-life/ln(2). Default 0.5: with LAG_BC=1 the self-excitation loop is broken so stability no longer constrains half-life; sim shows 0.5-0.7 recovers the compensation strength that BC removes (BC at h=0.3 yields only ~half the physical lag angle in steady state). Tune on site, range 0.3-0.8. 仅 UATK_VEL_USE=0（或速度无效回退）时使用；VEL_USE=1 时该半衰期不参与补偿。
    // @Units: s
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("LAG_P_HL", 34, FD_Attack_Copter, lag_pitch_half_life_s, 0.5f),

    // @Param: LAG_PMAX
    // @DisplayName: Pitch lag compensation limit
    // @Description: Maximum absolute value of the pitch lag compensation offset added to the camera pitch angle error in Stage 3
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAG_PMAX", 35, FD_Attack_Copter, lag_pitch_max_deg, 15.0f),

    // @Param: LAG_Y_HL
    // @DisplayName: Yaw lag compensation half-life
    // @Description: Half-life in seconds of the leaky integrator estimating how far the velocity direction lags the camera yaw axis (sideslip). 0 disables yaw compensation. Effective time constant is half-life/ln(2); expect roughly 0.6*flight_time seconds. Tune on site. 仅 UATK_VEL_USE=0（或速度无效回退）时使用；VEL_USE=1 时该半衰期不参与补偿。
    // @Units: s
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("LAG_Y_HL", 36, FD_Attack_Copter, lag_yaw_half_life_s, 0.3f),

    // @Param: LAG_YMAX
    // @DisplayName: Yaw lag compensation limit
    // @Description: Maximum absolute value of the yaw lag compensation offset added to the camera yaw angle error in Stage 3
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAG_YMAX", 37, FD_Attack_Copter, lag_yaw_max_deg, 15.0f),

    // @Param: LAG_BC
    // @DisplayName: Lag compensation back-calculation enable
    // @Description: The lag estimator integrates the previous COMMANDED rate (pure decaying feedforward on the command). This switch subtracts the compensator's own contribution (TRC*offset, only in LOS_RATE stage where the offset is actually injected) from the integrator input. MUST stay 1: with 0 the command->offset->command positive feedback is direct and the offset diverges/oscillates. 0 exists only for A/B comparison. 仅积分法（UATK_VEL_USE=0 或速度无效回退）时有效。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LAG_BC", 38, FD_Attack_Copter, lag_bc_en, 1),

    // @Param: FF_ATK
    // @DisplayName: Attack pitch rate feedforward gain
    // @Description: 离架确认后注入 pitch 速率环的静态速率前馈增益（AC_PID kff，单位同 ATC_RAT_PIT_FF）。平时（悬停/自稳/锁架期/退出 ATTACK 后）恒为 0。0=禁用。要求 ATC_RAT_PIT_FF 保持 0，避免双控制源。
    // @Range: 0 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("FF_ATK", 39, FD_Attack_Copter, ff_atk, 0.0f),

    // @Param: AUTO
    // @DisplayName: CHASE auto attack
    // @Description: 1 = CHASE 模式下接近目标到 UATK_AUTO_DST 以内自动切打击模式
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTO", 40, FD_Attack_Copter, auto_attack, 0),

    // @Param: AUTO_DST
    // @DisplayName: CHASE auto attack distance
    // @Description: CHASE 自动切打击模式的距离阈值
    // @Units: m
    // @Range: 10 5000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("AUTO_DST", 41, FD_Attack_Copter, auto_attack_dist, 500.0f),

    // @Param: CH_ALT
    // @DisplayName: CHASE takeoff altitude
    // @Description: CHASE 模式起飞目标高度（相对 home）
    // @Units: m
    // @Range: 5 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CH_ALT", 42, FD_Attack_Copter, chase_takeoff_alt, 30.0f),

    // @Param: CH_DH
    // @DisplayName: CHASE relative altitude to target
    // @Description: CHASE 模式朝目标飞行时拦截机与目标的相对高度差（拦截机高度减目标高度）。正值=拦截机比目标高，负值=拦截机比目标低
    // @Units: m
    // @Range: -200 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CH_DH", 43, FD_Attack_Copter, chase_alt_dh, -20.0f),

    // @Param: CH_MIN
    // @DisplayName: CHASE minimum altitude above home
    // @Description: CHASE 模式朝目标飞行时的最低高度下限（相对起飞点），高度下偏后不得低于此值
    // @Units: m
    // @Range: 0 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CH_MIN", 44, FD_Attack_Copter, chase_alt_min, 40.0f),

    // @Param: VEL_USE
    // @DisplayName: Velocity based lag compensation enable
    // @Description: 1 = 用本机 EKF 速度方向做滞后补偿（直接法 update_lag_offset_direct：光轴相对速度方向的夹角，按 UATK_LAG_PMAX/UATK_LAG_YMAX 限幅），并在 LOS_RATE 阶段屏蔽 FWD_PIT；0 = 不用速度，回退半衰期积分法（UATK_LAG_*_HL / UATK_LAG_BC 生效，FWD_PIT 按 UATK_FWD_EN 正常叠加）。需置 1 才启用速度补偿；position_ok 无效或地速 <5 m/s 时自动回退积分法。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VEL_USE", 45, FD_Attack_Copter, vel_use, 1),
    AP_GROUPEND
};

FD_Attack_Copter::FD_Attack_Copter()
{
    _camera_e_deg.zero();
    _velocity_c_deg.zero();
    _velocity_speed_ms = 0.0f;
    _velocity_valid = false;
    _target_rate_c_dps.zero();
    _target_rate_b_dps.zero();
    _guid_rate_c_dps.zero();
    _lag_offset_pitch_deg = 0.0f;
    _lag_offset_yaw_deg = 0.0f;
    _control_stage = ControlStage::NONE;

    los_yaw_rate_pid.reset_I();
    los_yaw_rate_pid.reset_filter();
    los_pitch_rate_pid.reset_I();
    los_pitch_rate_pid.reset_filter();
}

// 控制量合成（无参：输入取基类备好的 _attitude_b_deg / _observation_dt_s /
// _have_observation）。仍按相机帧率更新，速率环 dt 保持帧间隔。
void FD_Attack_Copter::update_control()
{
    if (!_have_observation) {
        return;     // 无新观测帧时保持上一帧输出
    }

    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(_attitude_b_deg.x),
                                      radians(_attitude_b_deg.y),
                                      radians(_attitude_b_deg.z));

    const Matrix3f rotation_ned_from_camera =
        rotation_ned_from_body * rotation_camera_to_body;

    float camera_roll_e_rad_internal;
    float camera_pitch_e_rad_internal;
    float camera_yaw_e_rad_internal;
    rotation_ned_from_camera.to_euler(&camera_roll_e_rad_internal,
                                      &camera_pitch_e_rad_internal,
                                      &camera_yaw_e_rad_internal);

    _camera_e_deg.x = degrees(camera_roll_e_rad_internal);
    _camera_e_deg.y = degrees(camera_pitch_e_rad_internal);
    _camera_e_deg.z = degrees(camera_yaw_e_rad_internal);

    Vector3f velocity_ned_ms(0.0f, 0.0f, 0.0f);
    const bool use_velocity = vel_use.get() == 1;
    const bool have_velocity = AP::ahrs().get_velocity_NED(velocity_ned_ms);
    const bool velocity_finite = isfinite(velocity_ned_ms.x) &&
                                 isfinite(velocity_ned_ms.y) &&
                                 isfinite(velocity_ned_ms.z);
    _velocity_speed_ms = have_velocity && velocity_finite ?
                         velocity_ned_ms.length() : 0.0f;
    _velocity_valid = use_velocity &&
                      copter.position_ok() &&
                      have_velocity &&
                      velocity_finite &&
                      (_velocity_speed_ms > 5.0f);
    if (_velocity_valid) {
        // 速度方向在相机系里的角度（未钳位），滞后补偿/日志/调试打印共用
        // （rotation_ned_from_camera 已用延迟姿态算好，直接复用）
        Matrix3f rotation_camera_from_ned = rotation_ned_from_camera;
        rotation_camera_from_ned.transpose();
        const Vector3f velocity_camera_ms = rotation_camera_from_ned * velocity_ned_ms;
        _velocity_c_deg.x = degrees(atan2f(velocity_camera_ms.y, velocity_camera_ms.x));
        _velocity_c_deg.y = degrees(atan2f(-velocity_camera_ms.z,
                                           velocity_camera_ms.xy().length()));
    } else {
        _velocity_c_deg.zero();
    }

    const float camera_pitch_error_abs_deg = fabsf(_target_c_deg.y);
    const float camera_yaw_error_abs_deg = fabsf(_target_c_deg.x);

    if (_observation_dt_s > 0.0f) {
        if (_enable_lag_offset) {
            // 有本机速度（position_ok）时直接用光轴与速度方向之差，否则回退半衰期积分
            if (_velocity_valid) {
                update_lag_offset_direct();
            } else {
                update_lag_offset_integral(_observation_dt_s);
            }
        } else {
            _lag_offset_pitch_deg = 0.0f;
            _lag_offset_yaw_deg = 0.0f;
        }
    }
    const float camera_yaw_p_rate_dps = yaw_angle_gain.get() * _target_c_deg.x;
    const float camera_pitch_p_rate_dps = pitch_angle_gain.get() * _target_c_deg.y;
    const float camera_level_rate_dps = fabsf(_camera_e_deg.y) < 70.0f ?
                                        -roll_level_gain.get() * _camera_e_deg.x :
                                        0.0f;

    const float rate_limit_dps_value = MAX(rate_limit_dps.get(), 10.0f);
    const float visibility_limit_deg_value =
        MAX(visibility_limit_deg.get(), 0.0f);
    const bool target_outside_visibility =
        (camera_pitch_error_abs_deg > visibility_limit_deg_value) ||
        (camera_yaw_error_abs_deg > visibility_limit_deg_value);

    if (camera_pitch_error_abs_deg > 60.0f) {
        if (_control_stage != ControlStage::PITCH_CAPTURE) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack stage PITCH_CAPTURE");
        }
        _control_stage = ControlStage::PITCH_CAPTURE;
        if (los_rate_control_active) {
            los_yaw_rate_pid.reset_I();
            los_yaw_rate_pid.reset_filter();
            los_pitch_rate_pid.reset_I();
            los_pitch_rate_pid.reset_filter();
            los_rate_control_active = false;
        }
        _target_rate_c_dps.x = 0.0f;
        _target_rate_c_dps.y = constrain_float(
            camera_pitch_p_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _target_rate_c_dps.z = 0.0f;
        _guid_rate_c_dps = _target_rate_c_dps;   // 捕获阶段无补偿注入，基础指令=最终指令
    } else if (target_outside_visibility) {
        if (_control_stage != ControlStage::ANGLE_CAPTURE) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack stage ANGLE_CAPTURE");
        }
        _control_stage = ControlStage::ANGLE_CAPTURE;
        if (los_rate_control_active) {
            los_yaw_rate_pid.reset_I();
            los_yaw_rate_pid.reset_filter();
            los_pitch_rate_pid.reset_I();
            los_pitch_rate_pid.reset_filter();
            los_rate_control_active = false;
        }
        _target_rate_c_dps.x = constrain_float(
            camera_level_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _target_rate_c_dps.y = constrain_float(
            camera_pitch_p_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _target_rate_c_dps.z = constrain_float(
            camera_yaw_p_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _guid_rate_c_dps = _target_rate_c_dps;   // 捕获阶段无补偿注入，基础指令=最终指令
    } else {
        if (_control_stage != ControlStage::LOS_RATE) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack stage LOS_RATE");
        }
        _control_stage = ControlStage::LOS_RATE;
        if (!los_rate_control_active) {
            los_yaw_rate_pid.reset_I();
            los_yaw_rate_pid.reset_filter();
            los_pitch_rate_pid.reset_I();
            los_pitch_rate_pid.reset_filter();
            los_rate_control_active = true;
        }

        const float track_yaw_base_dps = track_yaw_gain.get() * _target_c_deg.x;
        // 有位置（用速度做滞后补偿）时不再叠加 FWD_PIT，避免重复的 pitch 补偿
        const float track_pitch_bias_deg =
            (_velocity_valid || (forward_pitch_en.get() == 0)) ? 0.0f : forward_pitch_deg.get();
        const float track_pitch_base_dps =
            track_pitch_gain.get() * (_target_c_deg.y + track_pitch_bias_deg);
        _guid_rate_c_dps.x = constrain_float(
            camera_level_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _guid_rate_c_dps.y = constrain_float(
            -los_pitch_rate_pid.update_all(0, _los_c_rate.y, _observation_dt_s) + track_pitch_base_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _guid_rate_c_dps.z = constrain_float(
            -los_yaw_rate_pid.update_all(0, _los_c_rate.z, _observation_dt_s) + track_yaw_base_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _target_rate_c_dps = _guid_rate_c_dps;
        _target_rate_c_dps.y = constrain_float(
            _guid_rate_c_dps.y + track_pitch_gain.get() * _lag_offset_pitch_deg,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _target_rate_c_dps.z = constrain_float(
            _guid_rate_c_dps.z + track_yaw_gain.get() * _lag_offset_yaw_deg,
            -rate_limit_dps_value,
            rate_limit_dps_value);
    }

    apply_target_visibility_limit();
    _target_rate_b_dps = rotation_camera_to_body * _target_rate_c_dps;
}

// 可见性限制：离轴已到边界时，把继续把目标往外推的速率分量清零
void FD_Attack_Copter::apply_target_visibility_limit()
{
    const float limit_deg = MAX(visibility_limit_deg.get(), 0.0f);
    const float yaw_los_error_deg = _target_c_deg.x;
    const float pitch_los_error_deg = _target_c_deg.y;

    if ((yaw_los_error_deg > limit_deg) && (_target_rate_c_dps.z < 0.0f)) {
        _target_rate_c_dps.z = 0.0f;
    } else if ((yaw_los_error_deg < -limit_deg) && (_target_rate_c_dps.z > 0.0f)) {
        _target_rate_c_dps.z = 0.0f;
    }

    if ((pitch_los_error_deg > limit_deg) && (_target_rate_c_dps.y < 0.0f)) {
        _target_rate_c_dps.y = 0.0f;
    } else if ((pitch_los_error_deg < -limit_deg) && (_target_rate_c_dps.y > 0.0f)) {
        _target_rate_c_dps.y = 0.0f;
    }
}

void FD_Attack_Copter::set_lag_offset_enabled(bool en)
{
    _enable_lag_offset = en;
    if (!en) {
        // 关闭时确定性归零，不依赖下一帧观测
        _lag_offset_pitch_deg = 0.0f;
        _lag_offset_yaw_deg = 0.0f;
    }
}

// 直接法滞后估计——相机光轴相对速度方向的滞后角（有本机速度时用）。
// 与 _target_c_deg 同帧同口径：相机系 X=光轴（机体 -Z，即推力方向）、Y=右、Z=下，
// 角度形式与 FD_Target_HY::pixel_offset_to_spherical_deg() 一致。
void FD_Attack_Copter::update_lag_offset_direct()
{
    if (!_velocity_valid) {
        return;
    }
    const float lag_pitch_max = MAX(lag_pitch_max_deg.get(), 0.0f);
    const float lag_yaw_max = MAX(lag_yaw_max_deg.get(), 0.0f);
    // 速度方向角已在 update_control() 按相机系算好（_velocity_c_deg，未钳位）；
    // 符号与积分法同向——相机轴领先速度方向为正（光轴在速度方向右侧/上方为正）
    _lag_offset_yaw_deg = constrain_float(-_velocity_c_deg.x, -lag_yaw_max, lag_yaw_max);
    _lag_offset_pitch_deg = constrain_float(-_velocity_c_deg.y, -lag_pitch_max, lag_pitch_max);
}

// 积分法滞后估计（无本机速度时回退，UATK_LAG_* 参数生效）。
// 对历史指令速率做泄漏积分（半衰期 LAG_*_HL）；LAG_BC=1 时用 _guid_rate_c_dps
// （不含补偿器自身注入的 TRC*offset 项），打断"指令→偏置→指令"的正反馈。
void FD_Attack_Copter::update_lag_offset_integral(float observation_dt_s)
{
    const float lag_decay_pitch = (lag_pitch_half_life_s.get() > 0.001f) ?
        powf(0.5f, observation_dt_s / lag_pitch_half_life_s.get()) : 0.0f;
    const float lag_decay_yaw = (lag_yaw_half_life_s.get() > 0.001f) ?
        powf(0.5f, observation_dt_s / lag_yaw_half_life_s.get()) : 0.0f;
    const Vector3f &lag_ref_dps = (lag_bc_en.get() != 0) ?
        _guid_rate_c_dps : _target_rate_c_dps;
    _lag_offset_pitch_deg =
        (_lag_offset_pitch_deg + lag_ref_dps.y * observation_dt_s) * lag_decay_pitch;
    _lag_offset_yaw_deg =
        (_lag_offset_yaw_deg + lag_ref_dps.z * observation_dt_s) * lag_decay_yaw;
    const float lag_pitch_max = MAX(lag_pitch_max_deg.get(), 0.0f);
    const float lag_yaw_max = MAX(lag_yaw_max_deg.get(), 0.0f);
    _lag_offset_pitch_deg = constrain_float(_lag_offset_pitch_deg,
                                            -lag_pitch_max, lag_pitch_max);
    _lag_offset_yaw_deg = constrain_float(_lag_offset_yaw_deg,
                                          -lag_yaw_max, lag_yaw_max);
}

// 控制侧日志（100ms 定时由基类 update_log() 统一把关，这里不自己计时）：
//   UAT5 = 最终 LOS rate 相机系三分量（从基类成员 _los_c_rate 读）+ 控制量（TCR/TCP/TCY）
//   UAT6 = 机体系目标角速率（送姿态控制器）
//   UAT7 = 滞后偏置（OfP/OfY）+ 相机系速度方向角（VcP/VcY）+ 速度有效位（Val）+ 地速（Spd）
void FD_Attack_Copter::update_control_log()
{
    AP::logger().WriteStreaming(
        "UAT5",
        "TimeUS,LcXr,LcYr,LcZr,TCR,TCP,TCY",
        "skkkkkk",
        "F------",
        "Qffffff",
        AP_HAL::micros64(),
        _los_c_rate.x,
        _los_c_rate.y,
        _los_c_rate.z,
        _target_rate_c_dps.x,
        _target_rate_c_dps.y,
        _target_rate_c_dps.z);

    AP::logger().WriteStreaming(
        "UAT6",
        "TimeUS,TBR,TBP,TBY",
        "skkk",
        "F---",
        "Qfff",
        AP_HAL::micros64(),
        _target_rate_b_dps.x,
        _target_rate_b_dps.y,
        _target_rate_b_dps.z);

    AP::logger().WriteStreaming(
        "UAT7",
        "TimeUS,OfP,OfY,VcP,VcY,Val,Spd",
        "sdddd-n",
        "F------",
        "QffffBf",
        AP_HAL::micros64(),
        _lag_offset_pitch_deg,
        _lag_offset_yaw_deg,
        _velocity_c_deg.y,
        _velocity_c_deg.x,
        (uint8_t)(_velocity_valid ? 1 : 0),
        _velocity_speed_ms);
}

// 控制侧调试打印（bit0 相机姿态 / bit3 目标速率 / bit4 机体系目标速率 /
// bit5 控制阶段与离轴角 / bit7 速度方向补偿）。基类负责 bit0 前半、bit1/2/6/8，
// 1Hz 调用节拍也由基类调度。
void FD_Attack_Copter::do_print_control(int16_t print_mask)
{
    if (print_mask & (1U << 0)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Cam R/P/Y:%.1f/%.1f/%.1f",
                        _camera_e_deg.x,
                        _camera_e_deg.y,
                        _camera_e_deg.z);
    }
    if (print_mask & (1U << 3)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK TcR xyz:%.1f/%.1f/%.1f",
                        _target_rate_c_dps.x,
                        _target_rate_c_dps.y,
                        _target_rate_c_dps.z);
    }
    if (print_mask & (1U << 4)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK TbR xyz:%.1f/%.1f/%.1f src:%u",
                        _target_rate_b_dps.x,
                        _target_rate_b_dps.y,
                        _target_rate_b_dps.z,
                        (unsigned)_target_source);
    }
    if (print_mask & (1U << 5)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Stg:%u DY/DP:%.1f/%.1f",
                        (unsigned)_control_stage,
                        _target_c_deg.x,
                        _target_c_deg.y);
    }
    if (print_mask & (1U << 7)) {
        // 速度方向补偿调试：开关状态、补偿角、相机系速度方向角（未钳位）、地速
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK VU:%u OfP/OfY:%.1f/%.1f",
                        (unsigned)(vel_use.get() != 0 ? 1 : 0),
                        _lag_offset_pitch_deg,
                        _lag_offset_yaw_deg);
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK VcP/VcY:%.1f/%.1f Spd:%.1f",
                        _velocity_c_deg.y,
                        _velocity_c_deg.x,
                        _velocity_speed_ms);
    }
}

// 外部目标注入（CHASE 收到的 31011 靶机 / AP_Follow，经 set_target_loc 落地）。
// 基类 update_targets() 在 target_loc 已初始化时调用本函数，之后再调 target_loc.update()。
void FD_Attack_Copter::update_external_targets()
{
    if (copter.uchase.have_target()) {
        Location follow_target_loc = copter.uchase.get_target_loc();
        // 按 31011 的靶机速度把目标位置前推 0.1s（口径与下方 follow 分支一致）
        const Vector3f target_vel_ned_ms = copter.uchase.get_target_vel_ned_ms();
        const Vector3p prediction_offset_ned_m(target_vel_ned_ms.x * 0.1f,
                                               target_vel_ned_ms.y * 0.1f,
                                               target_vel_ned_ms.z * 0.1f);
        follow_target_loc.offset(prediction_offset_ned_m);
        target_loc->set_target_loc(follow_target_loc);
    }
#if MODE_FOLLOW_ENABLED
    else if ((target_loc->use_external_loc.get() == 1) && (copter.g2.follow.have_target())) {
        Location follow_target_loc;
        Vector3f follow_target_velocity_ned_ms;
        if (copter.g2.follow.get_target_location_and_velocity(follow_target_loc,
                                                              follow_target_velocity_ned_ms)) {
            // AP_Follow 按 FOLL_ALT_TYPE 返回帧（本板默认 ABOVE_HOME），全局目标统一存海拔(MSL)；
            // 换算失败（home 未建立）不注入，避免相对高度混进全局目标
            if (follow_target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
                const Vector3p prediction_offset_ned_m(follow_target_velocity_ned_ms.x * 0.1f,
                                                       follow_target_velocity_ned_ms.y * 0.1f,
                                                       follow_target_velocity_ned_ms.z * 0.1f);
                follow_target_loc.offset(prediction_offset_ned_m);
                target_loc->set_target_loc(follow_target_loc);
            }
        }
    }
#endif
}

// 源切换时控制侧复位：只复位两个速率环的 I 与滤波器，并放掉 LOS_RATE 激活标志
// （LOS/EKF 侧由基类 reset_chains() 处理）
void FD_Attack_Copter::on_source_changed()
{
    los_yaw_rate_pid.reset_I();
    los_yaw_rate_pid.reset_filter();
    los_pitch_rate_pid.reset_I();
    los_pitch_rate_pid.reset_filter();
    los_rate_control_active = false;
}

// 目标丢失/清输出时控制侧复位：阶段归 NONE + 相机姿态/速度/目标速率/制导基础量/
// 滞后偏置清零 + 速率环复位（每周期可能被反复调用，全部操作幂等）
void FD_Attack_Copter::on_target_lost()
{
    if (_control_stage != ControlStage::NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "UAttack stage NONE");
    }
    _camera_e_deg.zero();
    _velocity_c_deg.zero();
    _velocity_speed_ms = 0.0f;
    _velocity_valid = false;
    _target_rate_c_dps.zero();
    _target_rate_b_dps.zero();
    _guid_rate_c_dps.zero();
    _lag_offset_pitch_deg = 0.0f;
    _lag_offset_yaw_deg = 0.0f;
    _control_stage = ControlStage::NONE;
    los_yaw_rate_pid.reset_I();
    los_yaw_rate_pid.reset_filter();
    los_pitch_rate_pid.reset_I();
    los_pitch_rate_pid.reset_filter();
}
