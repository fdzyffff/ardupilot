#include "Copter.h"

const AP_Param::GroupInfo UAttack::var_info[] = {
    AP_GROUPINFO("TCAM_USE", 0, UAttack, use_target_cam, 0),
    AP_GROUPINFO("TLOC_USE", 1, UAttack, use_target_loc, 0),
    AP_GROUPINFO("CAM_DLY", 2, UAttack, camera_delay_ms, 0),
    AP_GROUPINFO("FILT_HZ", 3, UAttack, los_vector_filt_hz, 2.0f),
    AP_GROUPINFO("YAW_P", 7, UAttack, yaw_angle_gain, 1.0f),
    AP_GROUPINFO("PIT_P", 8, UAttack, pitch_angle_gain, 1.0f),
    AP_GROUPINFO("RATE_MAX", 9, UAttack, rate_limit_dps, 30.0f),
    AP_GROUPINFO("VIS_LIM", 10, UAttack, visibility_limit_deg, 20.0f),
    AP_SUBGROUPINFO(target_loc, "TL_", 11, UAttack, FD_Target_Loc),
    AP_SUBGROUPINFO(target_cam, "TC_", 12, UAttack, FD_Target_HY),
    AP_SUBGROUPINFO(los_yaw_rate_pid, "NAV_Y_", 13, UAttack, AC_PID),
    AP_SUBGROUPINFO(los_pitch_rate_pid, "NAV_P_", 14, UAttack, AC_PID),
    AP_GROUPINFO("DEBUG", 15, UAttack, debug_print, 0),

    // @Param: THR
    // @DisplayName: Attack throttle
    // @Description: Normalized throttle target used in Attack mode
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("THR", 16, UAttack, throttle, 0.5f),

    // @Param: THR_RATE
    // @DisplayName: Attack throttle change rate
    // @Description: Maximum normalized throttle change per second in Attack mode
    // @Units: 1/s
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("THR_RATE", 17, UAttack, throttle_rate, 0.1f),

    // @Param: RLL_P
    // @DisplayName: Camera earth-level roll gain
    // @Description: Camera roll rate gain used while camera pitch is below 80 degrees
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("RLL_P", 18, UAttack, roll_level_gain, 1.0f),

    // @Param: FWD_PIT
    // @DisplayName: Stage 3 forward pitch angle
    // @Description: Camera earth-frame pitch target used to add forward acceleration in Stage 3
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FWD_PIT", 19, UAttack, forward_pitch_deg, 30.0f),

    // @Param: TRC_Y
    // @DisplayName: Attack track yaw gain
    // @Description: Proportional pursuit gain on camera yaw angle error, added to LOS-rate PID output in Stage 3
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRC_Y", 20, UAttack, track_yaw_gain, 1.0f),

    // @Param: TRC_P
    // @DisplayName: Attack track pitch gain
    // @Description: Proportional pursuit gain on camera pitch angle error, added to LOS-rate PID output in Stage 3
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRC_P", 21, UAttack, track_pitch_gain, 1.0f),

    // @Param: FWD_EN
    // @DisplayName: Attack forward pitch rate enable
    // @Description: When set to 0 the camera pitch forward rate term in Stage 3 is forced to 0 (disabled); when non-zero the term is enabled
    // @User: Advanced
    AP_GROUPINFO("FWD_EN", 22, UAttack, forward_pitch_en, 0),

    // @Param: LAG_P_HL
    // @DisplayName: Pitch lag compensation half-life
    // @Description: Half-life in seconds of the leaky integrator estimating how far the velocity direction lags the camera pitch axis (sideslip). 0 disables pitch compensation. Effective time constant is half-life/ln(2). Default 0.5: with LAG_BC=1 the self-excitation loop is broken so stability no longer constrains half-life; sim shows 0.5-0.7 recovers the compensation strength that BC removes (BC at h=0.3 yields only ~half the physical lag angle in steady state). Tune on site, range 0.3-0.8.
    // @Units: s
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("LAG_P_HL", 23, UAttack, lag_pitch_half_life_s, 0.5f),

    // @Param: LAG_P_MAX
    // @DisplayName: Pitch lag compensation limit
    // @Description: Maximum absolute value of the pitch lag compensation offset added to the camera pitch angle error in Stage 3
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAG_P_MAX", 24, UAttack, lag_pitch_max_deg, 15.0f),

    // @Param: LAG_Y_HL
    // @DisplayName: Yaw lag compensation half-life
    // @Description: Half-life in seconds of the leaky integrator estimating how far the velocity direction lags the camera yaw axis (sideslip). 0 disables yaw compensation. Effective time constant is half-life/ln(2); expect roughly 0.6*flight_time seconds. Tune on site.
    // @Units: s
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("LAG_Y_HL", 25, UAttack, lag_yaw_half_life_s, 0.3f),

    // @Param: LAG_Y_MAX
    // @DisplayName: Yaw lag compensation limit
    // @Description: Maximum absolute value of the yaw lag compensation offset added to the camera yaw angle error in Stage 3
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAG_Y_MAX", 26, UAttack, lag_yaw_max_deg, 15.0f),

    // @Param: LAG_BC
    // @DisplayName: Lag compensation back-calculation enable
    // @Description: The lag estimator integrates the previous COMMANDED rate (pure decaying feedforward on the command). This switch subtracts the compensator's own contribution (TRC*offset, only in LOS_RATE stage where the offset is actually injected) from the integrator input. MUST stay 1: with 0 the command->offset->command positive feedback is direct and the offset diverges/oscillates. 0 exists only for A/B comparison.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LAG_BC", 27, UAttack, lag_bc_en, 1),

    // @Param: FF_ATK
    // @DisplayName: Attack pitch rate feedforward gain
    // @Description: 离架确认后注入 pitch 速率环的静态速率前馈增益（AC_PID kff，单位同 ATC_RAT_PIT_FF）。平时（悬停/自稳/锁架期/退出 ATTACK 后）恒为 0。0=禁用。要求 ATC_RAT_PIT_FF 保持 0，避免双控制源。
    // @Range: 0 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("FF_ATK", 28, UAttack, ff_atk, 0.0f),
    AP_GROUPEND
};

UAttack::UAttack()
{
    target_cam_initialized = false;
    target_loc_initialized = false;
    _target_source = TargetSource::NONE;
    _control_stage = ControlStage::NONE;
    have_last_target = false;
    los_rate_control_active = false;
    last_observation_ms = 0;
    last_filter_config_ms = 0;
    last_log_ms = 0;
    update_call_count = 0;
    update_rate_last_ms = 0;
    AP_Param::setup_object_defaults(this, var_info);
}

void UAttack::init()
{
    attitude_delay.init();
    gyro_delay.init();
    rotation_camera_to_body.from_euler(0.0f, radians(90.0f), 0.0f);
    rotation_body_to_camera.from_euler(0.0f, radians(90.0f), 0.0f);
    rotation_body_to_camera.transpose();

    _los_c_deg.zero();
    _los_e_deg.zero();
    _los_e_unit.zero();
    _los_e_dot.zero();
    _los_e_rate_dps.zero();
    _camera_e_deg.zero();
    _camera_log_e_deg.zero();
    _velocity_e_deg.zero();
    _velocity_speed_ms = 0.0f;
    _velocity_valid = false;
    _gyro_c_dps.zero();
    _los_c_rate_dps.zero();
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

    los_e_x_derivative.reset();
    los_e_y_derivative.reset();
    los_e_z_derivative.reset();
    los_e_x_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
    los_e_y_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
    los_e_z_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
    init_targets();
}

void UAttack::init_targets()
{
    target_cam_initialized = false;
    target_loc_initialized = false;

    if (use_target_cam.get() != 0) {
        target_cam_initialized = target_cam.init();
        if (!target_cam_initialized) {
            gcs().send_text(MAV_SEVERITY_WARNING, "UAttack HY unavailable");
        }
    }

    if (use_target_loc.get() != 0) {
        target_loc_initialized = target_loc.init();
        if (!target_loc_initialized) {
            gcs().send_text(MAV_SEVERITY_WARNING, "UAttack LOC unavailable");
        }
    }
}

void UAttack::update_delay_history()
{
    const Vector3f attitude_b_deg(degrees(AP::ahrs().get_roll_rad()),
                                  degrees(AP::ahrs().get_pitch_rad()),
                                  degrees(AP::ahrs().get_yaw_rad()));
    attitude_delay.push(attitude_b_deg);

    const Vector3f gyro_b_rads = AP::ahrs().get_gyro_latest();
    const Vector3f gyro_b_dps(degrees(gyro_b_rads.x),
                              degrees(gyro_b_rads.y),
                              degrees(gyro_b_rads.z));
    gyro_delay.push(gyro_b_dps);
}

bool UAttack::get_delayed_state(Vector3f &attitude_b_deg, Vector3f &gyro_b_dps) const
{
    const float loop_rate_hz = 100.0f;
    const uint16_t delay_steps = (uint16_t)constrain_float(
        camera_delay_ms.get() * loop_rate_hz * 0.001f,
        0.0f,
        (float)(UDelay::BUFFER_SIZE - 1));

    bool attitude_valid = attitude_delay.get_idx(delay_steps, attitude_b_deg);
    bool gyro_valid = gyro_delay.get_idx(delay_steps, gyro_b_dps);

    if (!attitude_valid) {
        attitude_b_deg = Vector3f(degrees(AP::ahrs().get_roll_rad()),
                                  degrees(AP::ahrs().get_pitch_rad()),
                                  degrees(AP::ahrs().get_yaw_rad()));
    }
    if (!gyro_valid) {
        const Vector3f gyro_b_rads = AP::ahrs().get_gyro_latest();
        gyro_b_dps = Vector3f(degrees(gyro_b_rads.x),
                              degrees(gyro_b_rads.y),
                              degrees(gyro_b_rads.z));
    }
    return attitude_valid && gyro_valid;
}

void UAttack::process_observation(const Vector2f &los_c_deg,
                                  const Vector3f &attitude_b_deg,
                                  const Vector3f &gyro_b_dps)
{
    _los_c_deg = los_c_deg;

    const float yaw_c_rad = radians(los_c_deg.x);
    const float pitch_c_rad = radians(los_c_deg.y);
    const Vector3f los_c_unit(cosf(pitch_c_rad) * cosf(yaw_c_rad),
                              cosf(pitch_c_rad) * sinf(yaw_c_rad),
                              -sinf(pitch_c_rad));

    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(attitude_b_deg.x),
                                      radians(attitude_b_deg.y),
                                      radians(attitude_b_deg.z));
    const Matrix3f rotation_ned_from_camera =
        rotation_ned_from_body * rotation_camera_to_body;
    _los_e_unit = rotation_ned_from_camera * los_c_unit;
    _los_e_unit.normalize();

    _los_e_deg.x = wrap_180(degrees(atan2f(_los_e_unit.y, _los_e_unit.x)));
    _los_e_deg.y = degrees(atan2f(-_los_e_unit.z, _los_e_unit.xy().length()));

    if (!have_last_target) {
        have_last_target = true;
        // 低通滤波已移到 _los_e_rate_dps 求导之后，首帧用样本自动初始化即可
        // los_e_x_filter.reset(_los_e_unit.x);
        // los_e_y_filter.reset(_los_e_unit.y);
        // los_e_z_filter.reset(_los_e_unit.z);
        los_e_x_filter.reset();
        los_e_y_filter.reset();
        los_e_z_filter.reset();
        los_e_x_derivative.reset();
        los_e_y_derivative.reset();
        los_e_z_derivative.reset();
    }

    const uint32_t now_ms = AP_HAL::millis();
    const float observation_dt_s = last_observation_ms == 0 ?
                                   0.01f :
                                   (float)(now_ms - last_observation_ms) * 0.001f;
    last_observation_ms = now_ms;

    // 低通滤波移到求导之后：先直接对单位向量分量求导，再对所得角速度滤波
    los_e_x_derivative.update(_los_e_unit.x, now_ms);
    los_e_y_derivative.update(_los_e_unit.y, now_ms);
    los_e_z_derivative.update(_los_e_unit.z, now_ms);

    _los_e_dot.x = los_e_x_derivative.slope() * 1000.0f;
    _los_e_dot.y = los_e_y_derivative.slope() * 1000.0f;
    _los_e_dot.z = los_e_z_derivative.slope() * 1000.0f;
    _los_e_dot -= _los_e_unit * (_los_e_unit * _los_e_dot);

    const Vector3f los_e_rate_rads = _los_e_unit % _los_e_dot;
    _los_e_rate_dps = Vector3f(los_e_x_filter.apply(degrees(los_e_rate_rads.x)),
                               los_e_y_filter.apply(degrees(los_e_rate_rads.y)),
                               los_e_z_filter.apply(degrees(los_e_rate_rads.z)));

    Matrix3f rotation_camera_from_ned = rotation_ned_from_camera;
    rotation_camera_from_ned.transpose();
    _los_c_rate_dps = rotation_camera_from_ned * _los_e_rate_dps;
    _gyro_c_dps = rotation_body_to_camera * gyro_b_dps;

    update_control_value(attitude_b_deg, observation_dt_s);
    update_log();
}

void UAttack::update_control_value(const Vector3f &attitude_b_deg, float observation_dt_s)
{
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(attitude_b_deg.x),
                                      radians(attitude_b_deg.y),
                                      radians(attitude_b_deg.z));

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

    const Matrix3f rotation_ned_from_camera_log =
        AP::ahrs().get_rotation_body_to_ned() * rotation_camera_to_body;
    float camera_roll_log_rad;
    float camera_pitch_log_rad;
    float camera_yaw_log_rad;
    rotation_ned_from_camera_log.to_euler(&camera_roll_log_rad,
                                          &camera_pitch_log_rad,
                                          &camera_yaw_log_rad);
    _camera_log_e_deg.x = degrees(camera_roll_log_rad);
    _camera_log_e_deg.y = degrees(camera_pitch_log_rad);
    _camera_log_e_deg.z = degrees(camera_yaw_log_rad);

    Vector3f velocity_ned_ms(0.0f, 0.0f, 0.0f);
    const bool have_velocity = AP::ahrs().get_velocity_NED(velocity_ned_ms);
    const bool velocity_finite = isfinite(velocity_ned_ms.x) &&
                                 isfinite(velocity_ned_ms.y) &&
                                 isfinite(velocity_ned_ms.z);
    _velocity_speed_ms = have_velocity && velocity_finite ?
                         velocity_ned_ms.length() : 0.0f;
    _velocity_valid = copter.position_ok() &&
                      have_velocity &&
                      velocity_finite &&
                      (_velocity_speed_ms > 5.0f);
    if (_velocity_valid) {
        _velocity_e_deg.x = degrees(atan2f(velocity_ned_ms.y, velocity_ned_ms.x));
        _velocity_e_deg.y = degrees(atan2f(-velocity_ned_ms.z,
                                           velocity_ned_ms.xy().length()));
    } else {
        _velocity_e_deg.zero();
    }

    const float camera_pitch_error_abs_deg = fabsf(_los_c_deg.y);
    const float camera_yaw_error_abs_deg = fabsf(_los_c_deg.x);

    if (observation_dt_s > 0.0f) {
        if (_enable_lag_offset) {
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
        } else {
            _lag_offset_pitch_deg = 0.0f;
            _lag_offset_yaw_deg = 0.0f;
        }
    }
    const float camera_yaw_p_rate_dps = yaw_angle_gain.get() * _los_c_deg.x;
    const float camera_pitch_p_rate_dps = pitch_angle_gain.get() * _los_c_deg.y;
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

        const float track_yaw_base_dps = track_yaw_gain.get() * _los_c_deg.x;
        const float track_pitch_bias_deg =
            (forward_pitch_en.get() == 0) ? 0.0f : forward_pitch_deg.get();
        const float track_pitch_base_dps =
            track_pitch_gain.get() * (_los_c_deg.y + track_pitch_bias_deg);
        _guid_rate_c_dps.x = constrain_float(
            camera_level_rate_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _guid_rate_c_dps.y = constrain_float(
            -los_pitch_rate_pid.update_all(0, _los_c_rate_dps.y, observation_dt_s) + track_pitch_base_dps,
            -rate_limit_dps_value,
            rate_limit_dps_value);
        _guid_rate_c_dps.z = constrain_float(
            -los_yaw_rate_pid.update_all(0, _los_c_rate_dps.z, observation_dt_s) + track_yaw_base_dps,
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

void UAttack::apply_target_visibility_limit()
{
    const float limit_deg = MAX(visibility_limit_deg.get(), 0.0f);
    const float yaw_los_error_deg = _los_c_deg.x;
    const float pitch_los_error_deg = _los_c_deg.y;

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

void UAttack::clear_target_output()
{
    if (_control_stage != ControlStage::NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "UAttack stage NONE");
    }
    _los_c_deg.zero();
    _los_e_deg.zero();
    _los_e_unit.zero();
    _los_e_dot.zero();
    _los_e_rate_dps.zero();
    _camera_e_deg.zero();
    _camera_log_e_deg.zero();
    _velocity_e_deg.zero();
    _velocity_speed_ms = 0.0f;
    _velocity_valid = false;
    _gyro_c_dps.zero();
    _los_c_rate_dps.zero();
    _target_rate_c_dps.zero();
    _target_rate_b_dps.zero();
    _guid_rate_c_dps.zero();
    _lag_offset_pitch_deg = 0.0f;
    _lag_offset_yaw_deg = 0.0f;
    _control_stage = ControlStage::NONE;
    have_last_target = false;
    los_rate_control_active = false;
    last_observation_ms = 0;
    los_yaw_rate_pid.reset_I();
    los_yaw_rate_pid.reset_filter();
    los_pitch_rate_pid.reset_I();
    los_pitch_rate_pid.reset_filter();
    los_e_x_derivative.reset();
    los_e_y_derivative.reset();
    los_e_z_derivative.reset();
}

void UAttack::update()
{
    update_call_count++;

    update_delay_history();

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_filter_config_ms >= 1000U) {
        los_e_x_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
        los_e_y_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
        los_e_z_filter.set_cutoff_frequency(100.0f, los_vector_filt_hz.get());
        last_filter_config_ms = now_ms;
    }

    if (target_cam_initialized) {
        target_cam.update();
    }
    if (target_loc_initialized) {
#if MODE_FOLLOW_ENABLED
        if ((target_loc.use_external_loc.get() == 1) && copter.g2.follow.have_target()) {
            Location follow_target_loc;
            Vector3f follow_target_velocity_ned_ms;
            if (copter.g2.follow.get_target_location_and_velocity(follow_target_loc,
                                                                  follow_target_velocity_ned_ms)) {
                const Vector3p prediction_offset_ned_m(follow_target_velocity_ned_ms.x * 0.1f,
                                                       follow_target_velocity_ned_ms.y * 0.1f,
                                                       follow_target_velocity_ned_ms.z * 0.1f);
                follow_target_loc.offset(prediction_offset_ned_m);
                target_loc.set_target_loc(follow_target_loc);
            }
        }
#endif
        target_loc.update();
    }

    TargetSource new_source = TargetSource::NONE;
    if (target_cam_initialized && target_cam.is_valid()) {
        new_source = TargetSource::CAM;
    } else if (target_loc_initialized && target_loc.is_valid()) {
        new_source = TargetSource::LOC;
    }

    if (new_source != _target_source) {
        if (new_source == TargetSource::CAM) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack source CAM");
        } else if (new_source == TargetSource::LOC) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack source LOC");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack no target");
        }
        _target_source = new_source;
        have_last_target = false;
        last_observation_ms = 0;
        los_rate_control_active = false;
        los_yaw_rate_pid.reset_I();
        los_yaw_rate_pid.reset_filter();
        los_pitch_rate_pid.reset_I();
        los_pitch_rate_pid.reset_filter();
    }

    if (_target_source == TargetSource::NONE) {
        clear_target_output();
        return;
    }

    Vector3f attitude_b_deg;
    Vector3f gyro_b_dps;
    get_delayed_state(attitude_b_deg, gyro_b_dps);

    Vector2f los_c_deg;
    bool have_observation = false;
    if (_target_source == TargetSource::CAM) {
        have_observation = target_cam.get_info(los_c_deg.x, los_c_deg.y);
    } else if (_target_source == TargetSource::LOC) {
        have_observation = target_loc.get_info(los_c_deg.x, los_c_deg.y);
    }

    if (have_observation) {
        process_observation(los_c_deg, attitude_b_deg, gyro_b_dps);
    }
}

void UAttack::handle_attack_msg(const mavlink_message_t &msg)
{
    if (target_loc_initialized) {
        target_loc.handle_msg(msg);
    }
    if (target_cam_initialized) {
        target_cam.handle_msg(msg);
    }
}

Location UAttack::get_target_loc()
{
    return target_loc.get_target_loc();
}

void UAttack::set_target_loc(Location &loc_in)
{
    if (target_loc_initialized) {
        target_loc.set_target_loc(loc_in);
    }
}

void UAttack::set_lag_offset_enabled(bool en)
{
    _enable_lag_offset = en;
    if (!en) {
        // 关闭时确定性归零，不依赖下一帧观测
        _lag_offset_pitch_deg = 0.0f;
        _lag_offset_yaw_deg = 0.0f;
    }
}

void UAttack::update_log()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 100U) {
        return;
    }
    last_log_ms = now_ms;

    AP::logger().WriteStreaming(
        "UAT1",
        "TimeUS,Src,LcY,LcP,LeY,LeP,CaR,CaP,CaY",
        "s-ddddddd",
        "F--------",
        "QBfffffff",
        AP_HAL::micros64(),
        (uint8_t)_target_source,
        _los_c_deg.x,
        _los_c_deg.y,
        _los_e_deg.x,
        _los_e_deg.y,
        _camera_e_deg.x,
        _camera_e_deg.y,
        _camera_e_deg.z);

    AP::logger().WriteStreaming(
        "UAT2",
        "TimeUS,LeX,LeY,LeZ,LdX,LdY,LdZ,Fps",
        "s------z",
        "F-------",
        "Qfffffff",
        AP_HAL::micros64(),
        _los_e_unit.x,
        _los_e_unit.y,
        _los_e_unit.z,
        _los_e_dot.x,
        _los_e_dot.y,
        _los_e_dot.z,
        target_cam.get_fps());

    AP::logger().WriteStreaming(
        "UAT3",
        "TimeUS,LcXr,LcYr,LcZr,TCR,TCP,TCY",
        "skkkkkk",
        "F------",
        "Qffffff",
        AP_HAL::micros64(),
        _los_c_rate_dps.x,
        _los_c_rate_dps.y,
        _los_c_rate_dps.z,
        _target_rate_c_dps.x,
        _target_rate_c_dps.y,
        _target_rate_c_dps.z);

    AP::logger().WriteStreaming(
        "UAT4",
        "TimeUS,TBR,TBP,TBY",
        "skkk",
        "F---",
        "Qfff",
        AP_HAL::micros64(),
        _target_rate_b_dps.x,
        _target_rate_b_dps.y,
        _target_rate_b_dps.z);

    AP::logger().WriteStreaming(
        "UAT5",
        "TimeUS,Valid,Spd,CaR,CaP,CaY,VeP,VeY",
        "s-nddddd",
        "F-------",
        "QBffffff",
        AP_HAL::micros64(),
        (uint8_t)_velocity_valid,
        _velocity_speed_ms,
        _camera_log_e_deg.x,
        _camera_log_e_deg.y,
        _camera_log_e_deg.z,
        _velocity_e_deg.y,
        _velocity_e_deg.x);
    AP::logger().WriteStreaming(
        "UAT6",
        "TimeUS,OfP,OfY",
        "sdd",
        "F--",
        "Qff",
        AP_HAL::micros64(),
        _lag_offset_pitch_deg,
        _lag_offset_yaw_deg);
}

void UAttack::do_print()
{
    const int16_t print_mask = debug_print.get();

    // measure actual update() call rate over the 1Hz print window
    const uint32_t now_ms = AP_HAL::millis();
    float update_rate_hz = 0.0f;
    if (update_rate_last_ms != 0) {
        const float dt_s = (now_ms - update_rate_last_ms) * 0.001f;
        if (dt_s > 0.0f) {
            update_rate_hz = update_call_count / dt_s;
        }
    }
    update_call_count = 0;
    update_rate_last_ms = now_ms;

    if (print_mask & (1U << 0)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Lc Y/P:%.1f/%.1f Le Y/P:%.1f/%.1f",
                        _los_c_deg.x,
                        _los_c_deg.y,
                        _los_e_deg.x,
                        _los_e_deg.y);
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Cam R/P/Y:%.1f/%.1f/%.1f",
                        _camera_e_deg.x,
                        _camera_e_deg.y,
                        _camera_e_deg.z);
        // FPS 打印移到 bit6，与 update 调用频率一起输出
        // gcs().send_text(MAV_SEVERITY_WARNING,
        //                 "ATK FPS:%.1f",
        //                 target_cam.get_fps());
    }
    if (print_mask & (1U << 1)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Le xyz:%.2f/%.2f/%.2f",
                        _los_e_unit.x,
                        _los_e_unit.y,
                        _los_e_unit.z);
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK LeD xyz:%.2f/%.2f/%.2f",
                        _los_e_dot.x,
                        _los_e_dot.y,
                        _los_e_dot.z);
    }
    if (print_mask & (1U << 2)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK LcR xyz:%.1f/%.1f/%.1f",
                        _los_c_rate_dps.x,
                        _los_c_rate_dps.y,
                        _los_c_rate_dps.z);
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
                        _los_c_deg.x,
                        _los_c_deg.y);
    }
    if (print_mask & (1U << 6)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK FPS:%.1f UpdHz:%.1f",
                        target_cam.get_fps(),
                        update_rate_hz);
    }
}


