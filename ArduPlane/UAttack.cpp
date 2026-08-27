#include "Plane.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_kr_pitch_pid   , "PTH_KR_", 0, UAttack, AC_PID),
    AP_GROUPINFO("PTH_KT",      1, UAttack, attack_kt_pitch,         1.0f),
    AP_GROUPINFO("PTH_KV",      2, UAttack, attack_kv_pitch,         1.0f),
    AP_GROUPINFO("PTH_LIM",     3, UAttack, pitch_limit,            30.f),
    AP_GROUPINFO("PTH_RLIM",    4, UAttack, pitch_rate_limit,       30.f),
    AP_GROUPINFO("PTH_OFF",     5, UAttack, attack_pitch_off,        0.0f),
    AP_GROUPINFO("YAW_KR",      6, UAttack, attack_kr_yaw,           0.0f),
    AP_GROUPINFO("YAW_KT",      7, UAttack, attack_kt_yaw,           1.0f),
    AP_GROUPINFO("YAW_KV",      8, UAttack, attack_kv_yaw,           1.0f),
    AP_SUBGROUPINFO(attack_kr_roll_pid    , "RLL_KR_", 9, UAttack, AC_PID),
    AP_GROUPINFO("RLL_KT",     10, UAttack, attack_kt_roll,          0.5f),
    AP_GROUPINFO("ANGLE",      11, UAttack, attack_angle,            0.f),
    AP_GROUPINFO("ANGLE_K",    12, UAttack, attack_k_angle,          1.0f),
    AP_GROUPINFO("THR",        13, UAttack, attack_throttle,        75.0f),
    AP_GROUPINFO("RTL_TOUT",   14, UAttack, atk_time_out,        20000),
    AP_GROUPINFO("DEBUG",      15, UAttack, print,                   0),
    AP_GROUPINFO("TCAM_USE",   16, UAttack, use_target_cam,          0),
    AP_GROUPINFO("TCAM_TYPE",  17, UAttack, use_target_cam_type,     1),
    AP_GROUPINFO("TLOC_USE",   18, UAttack, use_target_loc,          0),
    AP_GROUPINFO("FILT_Y_HZ",  19, UAttack, filt_yaw_hz,             2.0f),
    AP_GROUPINFO("FILT_P_HZ",  20, UAttack, filt_pithc_hz,           2.0f),

    AP_SUBGROUPPTR(_Target_ptr_loc,         "TL_",    21, UAttack,  FD_Target_Loc),
    AP_SUBGROUPPTR(_Target_ptr_cam_DYT,     "TC_",    22, UAttack,  FD_Target_DYT),

    AP_SUBGROUPINFO(attack_vely_pid    , "VELY_", 23, UAttack, AC_PID),
    AP_GROUPINFO("RLL_RLIM",   24, UAttack, roll_rate_limit,          45.0f),
    AP_GROUPINFO("RLL_LVL_K",  25, UAttack, roll_level_gain,           0.05f),
    AP_GROUPINFO("PTH_K1",     26, UAttack, attack_k1_pitch,            1.0f),
    AP_GROUPINFO("YAW_K1",     27, UAttack, attack_k1_yaw,              1.0f),
    AP_GROUPINFO("RLL_K1",     28, UAttack, attack_k1_roll,             1.0f),
    AP_GROUPEND
};

UAttack::UAttack()
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_align_angle = 0.0f;
    _align_angle_valid = false;
}

// initialise
void UAttack::init()
{
    udelay.init();
    _active = false;
    _angle_only_control = false;
    bf_info.x = 0.0f;
    bf_info.y = 0.0f;
    vel_bf_info.x = 0.0f;
    vel_bf_info.y = 0.0f;
    ef_info.x = 0.0f;
    ef_info.y = 0.0f;
    los_bf_rate.x = 0.0f;
    los_bf_rate.y = 0.0f;
    display_info.new_data = false;
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.p21 = 0.0f;
    display_info.p22 = 0.0f;
    display_info.p23 = 0.0f;
    display_info.p24 = 0.0f;
    display_info.count = 0;
    _target_pitch_rate = 0.0f;
    _target_yaw_rate = 0.0f;
    _target_roll_rate = 0.0f;
    _Target_ptr_cam = nullptr;
    _Target_ptr_loc = nullptr;
    _last_ms = millis();
    init_target();

    _los_e_unit_filter.set_cutoff_frequency(60.f, MIN(filt_yaw_hz.get(), filt_pithc_hz.get()));
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
}

void UAttack::update_control_value() {
    const float camera_spherical_angle_deg = degrees(acosf(constrain_float(cosf(radians(bf_info.y)) * cosf(radians(bf_info.x)), -1.0f, 1.0f)));
    const bool angle_only_control = is_active() && camera_spherical_angle_deg > 40.0f;
    if (angle_only_control != _angle_only_control) {
        _align_angle_valid = false;
        _align_angle_rate_filter.reset();
        attack_kr_roll_pid.reset_I();
        attack_kr_roll_pid.reset_filter();
    }
    _angle_only_control = angle_only_control;
    update_target_pitch_rate();
    update_target_yaw_rate();
    update_target_roll_rate();
    _last_ms = millis();
    update_log();
}

void UAttack::update_log() {
    AP::logger().WriteStreaming("UATK",
                                "TimeUS,bfx,bfy,efx,efy,efrx,efry,tpth,trll,tyaw",
                                "s---------",
                                "F---------",
                                "Qfffffffff",
                                AP_HAL::micros64(),
                                (float)bf_info.x,
                                (float)bf_info.y,
                                (float)ef_info.x,
                                (float)ef_info.y,
                                (float)los_bf_rate.x,
                                (float)los_bf_rate.y,
                                (float)_target_pitch_rate,
                                (float)_target_roll_rate,
                                (float)_target_yaw_rate);

    AP::logger().WriteStreaming("UAT2",
                                "TimeUS,type, angt,angm,agrt,agrm,vbfx,vbfy,bfex,bfey,dc",
                                "s----------",
                                "F----------",
                                "Qffffffffff",
                                AP_HAL::micros64(),
                                (float)current_idx,
                                (float)_attack_angle_target,
                                (float)_attack_angle_measure,
                                (float)_attack_angle_rate_target,
                                (float)_attack_angle_rate_measure,
                                (float)vel_bf_info.x,
                                (float)vel_bf_info.y,
                                (float)bfe_info.x,
                                (float)bfe_info.y,
                                (float)_delta_course);

    AP::logger().WriteStreaming("UAPH",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_kr_pitch_pid.get_pid_info().target,
                                (float)attack_kr_pitch_pid.get_pid_info().actual,
                                (float)attack_kr_pitch_pid.get_pid_info().FF,
                                (float)attack_kr_pitch_pid.get_pid_info().P,
                                (float)attack_kr_pitch_pid.get_pid_info().I,
                                (float)attack_kr_pitch_pid.get_pid_info().D,
                                (float)attack_kr_pitch_pid.get_pid_info().slew_rate,
                                (float)attack_kr_pitch_pid.get_pid_info().Dmod);

    AP::logger().WriteStreaming("UARL",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_kr_roll_pid.get_pid_info().target,
                                (float)attack_kr_roll_pid.get_pid_info().actual,
                                (float)attack_kr_roll_pid.get_pid_info().FF,
                                (float)attack_kr_roll_pid.get_pid_info().P,
                                (float)attack_kr_roll_pid.get_pid_info().I,
                                (float)attack_kr_roll_pid.get_pid_info().D,
                                (float)attack_kr_roll_pid.get_pid_info().slew_rate,
                                (float)attack_kr_roll_pid.get_pid_info().Dmod);

    AP::logger().WriteStreaming("UVEY",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_vely_pid.get_pid_info().target,
                                (float)attack_vely_pid.get_pid_info().actual,
                                (float)attack_vely_pid.get_pid_info().FF,
                                (float)attack_vely_pid.get_pid_info().P,
                                (float)attack_vely_pid.get_pid_info().I,
                                (float)attack_vely_pid.get_pid_info().D,
                                (float)attack_vely_pid.get_pid_info().slew_rate,
                                (float)attack_vely_pid.get_pid_info().Dmod);
}

const Vector2f& UAttack::get_bf_info() {
    return bf_info;
}

const Vector2f& UAttack::get_ef_info() {
    return ef_info;
}

const Vector2f& UAttack::get_los_bf_rate() {
    return los_bf_rate;
}

const Vector2f& UAttack::get_bfe_info() {
    return bfe_info;
}

void UAttack::init_target()
{
    bool use_cam = use_target_cam.get();
    bool use_loc = use_target_loc.get();

    if (use_cam) {
         // 1:mav
        if (use_target_cam_type.get() == 1) {
            _Target_ptr_cam_DYT = new FD_Target_DYT();
            if (_Target_ptr_cam_DYT->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target DYT init");
                _Target_ptr_cam = _Target_ptr_cam_DYT;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam, FD_Target_DYT::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target DYT Fail");
                _Target_ptr_cam_DYT = nullptr;
            }
        } 
        else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target CAM UNKNOW %d", use_target_cam_type.get());
            _Target_ptr_cam = nullptr;
        }
    }

    if (use_loc) {
        _Target_ptr_loc = new FD_Target_Loc();
        if (_Target_ptr_loc->init()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc init");
            AP_Param::load_object_from_eeprom(_Target_ptr_loc, FD_Target_Loc::var_info);
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc Fail");
            _Target_ptr_loc = nullptr;
        }
    }
}

// called at 100 Hz

void UAttack::update()
{
    udelay.push();
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
    }

    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->update();
    }

    if (_Target_ptr_loc != nullptr) {
        if (_Target_ptr_loc->use_external_loc.get() == 1) {
            if (plane.g2.follow.have_target()) {
                Location tmp_loc;
                Vector3f tmp_vel;
                if (plane.g2.follow.get_target_location_and_velocity(tmp_loc, tmp_vel)) {
                    Vector3p tmp_off = Vector3p(tmp_vel.x * 0.1f, tmp_vel.y * 0.1f, tmp_vel.z * 0.1f);
                    tmp_loc.offset(tmp_off);
                    _Target_ptr_loc->set_target_loc(tmp_loc);
                }
            }
        }
        _Target_ptr_loc->update();
    }


    // push frame angle to mission port
    {
        float gimbal_yaw = 0.0f;
        float gimbal_pitch = 0.0f;
        Location tmp_loc;
        if (_Target_ptr_cam != nullptr && _Target_ptr_cam->is_valid()) {
            if (_Target_ptr_cam->get_info(gimbal_yaw, gimbal_pitch)) {
                _Target_ptr_cam->recover_info();
            }
            tmp_loc = _Target_ptr_cam->get_target_loc();
        } else if (_Target_ptr_loc != nullptr && _Target_ptr_loc->is_valid()) {
            if (_Target_ptr_loc->get_info(gimbal_yaw, gimbal_pitch)) {
                _Target_ptr_loc->recover_info();
            }
            tmp_loc = _Target_ptr_loc->get_target_loc();
        }

        plane.uart.set_target_angle(gimbal_yaw, gimbal_pitch);
        plane.uart.set_target_loc(tmp_loc);
    } 

    if (_Target_ptr_cam != nullptr && _Target_ptr_cam->is_valid()) {
        if (current_idx < 2) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to CAM");
        }
        current_idx = 2;
    } else if (_Target_ptr_loc != nullptr && _Target_ptr_loc->is_valid()) {
        if (current_idx < 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to LOC");
        }
        current_idx = 1;
    } else {
        if (current_idx != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "No Valid Target");
        }
        current_idx = 0;

        _target_pitch_rate = 0.0f;
        _target_roll_rate = 0.0f;
        _target_yaw_rate = 0.0f;
        _align_angle_valid = false;
        _align_angle_rate_filter.reset();
    }

    float p1 = 0;
    float p2 = 0;
    if (current_idx == 1) {
        if (_Target_ptr_loc->get_info(p1, p2)) {
            handle_info(p1, p2);
        }
    }
    if (current_idx == 2) {
        if (_Target_ptr_cam->get_info(p1, p2)) {
            handle_info(p1, p2);
        }
    }
    if (current_idx > 0) {
        update_control_value();
        update_target_loc();
    }
    
    update_vel_bf_info();
}

void UAttack::update_vel_bf_info()
{
    Vector3f tmp_vel;
    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();
    if (plane.position_ok() && AP::ahrs().get_velocity_NED(tmp_vel)) {
        Matrix3f tmp_body_earth_m;
        tmp_body_earth_m.from_euler(_roll, _pitch, _yaw);
        Matrix3f tmp_earth_body_m = tmp_body_earth_m.transposed();
        Vector3f ef_unit = tmp_earth_body_m*tmp_vel;
        vel_bf_info.y = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
        vel_bf_info.x = wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));
    } else {
        vel_bf_info.zero();
    }

    if (plane.position_ok()) {
        _delta_course = wrap_180(AP::gps().ground_course() - degrees(AP::ahrs().get_yaw()));
    } else {
        _delta_course = 0.0f;
    }
}

void UAttack::handle_info(float p1, float p2) {

    display_info.p1 = p1;
    display_info.p2 = p2;

    // body fixed cam
    bf_info.x = p1; // yaw degree
    bf_info.y = p2; // pitch degree

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();
    // if (!udelay.get_idx(3-1, _roll, _pitch, _yaw)) {
    //     _roll = AP::ahrs().get_roll();
    //     _pitch = AP::ahrs().get_pitch();
    //     _yaw = AP::ahrs().get_yaw();
    // }

    {
        Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
        Matrix3f tmp_target_cam_m;
        tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
        Matrix3f tmp_cam_body_m;
        tmp_cam_body_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
        Matrix3f tmp_body_earthbody_m;
        tmp_body_earthbody_m.from_euler(_roll, _pitch, 0.0f);
        Matrix3f tmp_target_earth_m = tmp_body_earthbody_m*tmp_cam_body_m*tmp_target_cam_m;
        Vector3f ef_unit = tmp_target_earth_m*target_unit;

        float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
        float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));
        bfe_info.x = angle_yaw;
        bfe_info.y = angle_pitch;
    }



    float angle_pitch = 0.0f;
    float angle_yaw = 0.0f;

    Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_body_m;
    tmp_cam_body_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
    Matrix3f tmp_body_earth_m;
    tmp_body_earth_m.from_euler(_roll, _pitch, _yaw);
    Matrix3f tmp_target_earth_m = tmp_body_earth_m*tmp_cam_body_m*tmp_target_cam_m;
    Vector3f ef_unit = tmp_target_earth_m*target_unit;


    // static uint32_t last_info_ms = millis();
    // if (millis() - last_info_ms > 1000) {
    //     last_info_ms = millis();
    //     gcs().send_text(MAV_SEVERITY_INFO, "KKKKKK (%f, %f, %f)", ef_unit.x, ef_unit.y, ef_unit.z);
    //     gcs().send_text(MAV_SEVERITY_INFO, "VVVVVV (%f, %f, %f)", degrees(_roll), degrees(_pitch), degrees(_yaw));
    // }

    angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
    angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));


    ef_info.x = angle_yaw;
    ef_info.y = angle_pitch;

    const Vector3f los_e_filtered = _los_e_unit_filter.apply(ef_unit);
    Vector3f los_e_unit = los_e_filtered;
    if (!los_e_unit.is_zero()) {
        los_e_unit.normalize();
    }

    const uint32_t sample_ms = millis();
    _los_e_x_filter.update(los_e_unit.x, sample_ms);
    _los_e_y_filter.update(los_e_unit.y, sample_ms);
    _los_e_z_filter.update(los_e_unit.z, sample_ms);

    Vector3f los_e_unit_dot(_los_e_x_filter.slope() * 1000.0f,
                            _los_e_y_filter.slope() * 1000.0f,
                            _los_e_z_filter.slope() * 1000.0f);
    los_e_unit_dot -= los_e_unit * (los_e_unit * los_e_unit_dot);

    const Vector3f los_rate_e_rads = los_e_unit % los_e_unit_dot;
    const Matrix3f rotation_ned_to_body = AP::ahrs().get_rotation_body_to_ned().transposed();
    _los_rate_body_dps = rotation_ned_to_body * los_rate_e_rads;
    _los_rate_body_dps *= RAD_TO_DEG;

    los_bf_rate.x = _los_rate_body_dps.z;
    los_bf_rate.y = _los_rate_body_dps.y;

    display_info.new_data = true;
    display_info.count++;
}

// degree/second
void UAttack::update_target_pitch_rate() {
    float kt_pitch = attack_kt_pitch.get();
    float pitch_off = attack_pitch_off.get();
    float p = attack_k_angle.get();

    float dt = (millis() - _last_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}

    if (fabsf(bf_info.x) > 30.f) {
        _target_pitch_rate = 0.0f;
        attack_kr_pitch_pid.reset_I();
        attack_kr_pitch_pid.reset_filter();
        return;
    }
    // Use heading-level-frame elevation so bank angle does not appear as pitch error.
    float angle_err = constrain_float(bfe_info.y + pitch_off, -30.0f, 30.0f);

    if (_angle_only_control) {
        _target_pitch_rate = attack_k1_pitch.get() * angle_err;
        attack_kr_pitch_pid.reset_I();
        attack_kr_pitch_pid.reset_filter();
    } else {
        _attack_angle_target = attack_angle.get();
        _attack_angle_measure = -ef_info.y;
        _attack_angle_rate_target = (_attack_angle_target - _attack_angle_measure) * p;
        _attack_angle_rate_measure = -_los_rate_body_dps.y;

        // float attack_angle_rate_err = _attack_angle_rate_target - _attack_angle_rate_measure;

        // attack_angle_rate_err = constrain_float(attack_angle_rate_err, -30.0f, 30.0f);

        // _target_pitch_rate = k1_pitch * attack_angle_rate_err + k2_pitch * angle_err; // degrees/s

        _target_pitch_rate = attack_kr_pitch_pid.update_all(_attack_angle_rate_target, _attack_angle_rate_measure, dt) + kt_pitch * angle_err;

        if (plane.position_ok()) {
            float kv_pitch = attack_kv_pitch.get();
            float vel_angle_err = wrap_180(bf_info.y - vel_bf_info.y);
            _target_pitch_rate += vel_angle_err * kv_pitch;
        }
    }

    //Limit pitch rate
    float limit_pitch_rate = pitch_rate_limit;
    _target_pitch_rate = constrain_float(_target_pitch_rate, -limit_pitch_rate, limit_pitch_rate);

    // //Limit pitch
    float current_pitch = degrees(AP::ahrs().get_pitch());
    float limit_pitch = constrain_float(pitch_limit.get(), -60.f, 60.f);
    if (current_pitch > limit_pitch) {
        _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    } else if (current_pitch < -limit_pitch) {
        _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree/second
void UAttack::update_target_roll_rate() {
    float dt = (millis() - _last_ms) * 0.001f;
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.2f;
    }

    float kt_yaw = attack_kt_yaw.get();
    const float current_bf_yaw_rate = degrees(AP::ahrs().get_gyro().z);
    const float current_roll_deg = degrees(AP::ahrs().get_roll());
    const float current_pitch_deg = degrees(AP::ahrs().get_pitch());
    float angle_err = constrain_float(bf_info.x - (_angle_only_control ? 0.0f : _delta_course), -30.0f, 30.0f);

    float roll_level_rate = 0.0f;
    if (fabsf(current_pitch_deg) < 80.0f) {
        roll_level_rate = constrain_float(-current_roll_deg * roll_level_gain.get(), -5.0f, 5.0f);
    }

    if (_angle_only_control) {
        const float desired_roll_angle = constrain_float(angle_err, -45.0f, 45.0f);
        const float roll_angle_error = wrap_180(desired_roll_angle - current_roll_deg);
        _target_roll_rate = attack_k1_roll.get() * roll_angle_error;
        attack_kr_roll_pid.reset_I();
        attack_kr_roll_pid.reset_filter();
    } else {
        float desired_bf_yaw_rate = attack_kt_roll.get() * _los_rate_body_dps.z + kt_yaw * angle_err;
        _target_roll_rate = attack_kr_roll_pid.update_all(desired_bf_yaw_rate, current_bf_yaw_rate, dt);
    }

    _target_roll_rate += roll_level_rate;
    _target_roll_rate = constrain_float(_target_roll_rate, -roll_rate_limit.get(), roll_rate_limit.get());
}

// degree/second
void UAttack::update_target_yaw_rate() {
    float kt_yaw = attack_kt_yaw.get();
    // float boost_factor = constrain_float(fabsf(bf_info.x)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.x - (_angle_only_control ? 0.0f : _delta_course), -30.0f, 30.0f);
    if (_angle_only_control) {
        _target_yaw_rate = attack_k1_yaw.get() * angle_err;
    } else {
        _target_yaw_rate = attack_kr_yaw.get() * _los_rate_body_dps.z + kt_yaw * angle_err;

        if (plane.position_ok()) {
            float kv_yaw = attack_kv_yaw.get();
            float vel_angle_err = wrap_180(bf_info.x - vel_bf_info.x);
            _target_yaw_rate += vel_angle_err * kv_yaw;
        }
    }

    _target_yaw_rate = constrain_float(_target_yaw_rate, -30.0f, 30.0f);
    display_info.p11 = angle_err;
    display_info.p12 = kt_yaw;
    display_info.p13 = _target_yaw_rate;
    display_info.p14 = get_target_yaw_rate();
}

void UAttack::handle_attack_msg(const mavlink_message_t &msg) {
    if (_Target_ptr_loc != nullptr) {
        _Target_ptr_loc->handle_msg(msg);
    }
    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->handle_msg(msg);
    }
}

void UAttack::do_print()
{
    // put your 1Hz code here
    if ((print.get() & (1<<0)) && display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f, %0.0f, %0.0f, %0.0f", display_info.count_log, display_info.p1, display_info.p2, display_info.p3, display_info.p4);
        display_info.new_data = false;
    }
    if (print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f, %0.2f) on:%d", get_ef_info().x,get_ef_info().y, is_active());
    }
    if (print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f, %0.2f) on:%d", get_los_bf_rate().x,get_los_bf_rate().y, is_active());
    }
    if (print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "bfe_angle (%0.2f, %0.2f, %0.2f) on:%d", get_bfe_info().x,get_bfe_info().y, _delta_course, is_active());
    }
    if (print.get() & (1<<4)) { // 16
        gcs().send_text(MAV_SEVERITY_WARNING, "ar (%0.1f, %0.1f, %0.2f, %0.2f)", _attack_angle_target, _attack_angle_measure, _attack_angle_rate_target, _attack_angle_rate_measure);
    }
    if (print.get() & (1<<5)) { // 32
        gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f, %0.1f, %0.1f, %0.2f)", get_target_roll_rate(), get_target_pitch_rate(), get_target_yaw_rate(), attack_throttle.get());
    }
    if (print.get() & (1<<6)) { // 64
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f, %0.0f, %0.0f, %0.0f", display_info.p11, display_info.p12, display_info.p13, display_info.p14);
    }
}
