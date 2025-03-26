#include "Plane.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_roll_pid    , "ATKRLL_", 0, UAttack, AC_PID),
    AP_GROUPINFO("K1_PTH",      1, UAttack, attack_k1_pitch,         2.0f),
    AP_GROUPINFO("K2_PTH",      2, UAttack, attack_k2_pitch,         2.0f),
    AP_GROUPINFO("K1_YAW",      3, UAttack, attack_k1_yaw,           2.0f),
    AP_GROUPINFO("K2_YAW",      4, UAttack, attack_k2_yaw,           2.0f),
    AP_GROUPINFO("K_ANGLE",     5, UAttack, attack_k_angle,          1.0f),
    AP_GROUPINFO("THR",         6, UAttack, attack_throttle,        75.0f),
    AP_GROUPINFO("THR_RATE",    7, UAttack, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("OUTMS",       8, UAttack, attack_timeout,       2000),
    AP_GROUPINFO("ANGLE",       9, UAttack, attack_angle,           30.f),
    AP_GROUPINFO("PTH_LIM",    10, UAttack, pitch_limit,            30.f),
    AP_GROUPINFO("PTH_RLIM",   11, UAttack, pitch_rate_limit,       30.f),
    AP_GROUPINFO("OFF_PTH",    12, UAttack, attack_pitch_off,       -5.0f),
    AP_GROUPINFO("UPRINT",     13, UAttack, print,                   0),
    AP_GROUPINFO("TC_USE",     14, UAttack, use_target_cam,          0),
    AP_GROUPINFO("TL_USE",     15, UAttack, use_target_loc,          0),

    AP_SUBGROUPPTR(_Target_ptr_cam, "TC_",   16, UAttack,  FD_Target_K230),
    AP_SUBGROUPPTR(_Target_ptr_loc, "TL_",   17, UAttack,  FD_Target_Loc),
    AP_GROUPEND
};

UAttack::UAttack()
{
    AP_Param::setup_object_defaults(this, var_info);

    _yaw_sample_filter.set_cutoff_frequency(30.f, 2.f);
    _pitch_sample_filter.set_cutoff_frequency(30.f, 2.f);
    _last_yaw = 0.0f;
    _last_yaw_sample = 0.0f;
}

// initialise
void UAttack::init()
{
    udelay.init();
    _active = false;
    bf_info.x = 0.0f;
    bf_info.y = 0.0f;
    ef_info.x = 0.0f;
    ef_info.y = 0.0f;
    ef_rate_info.x = 0.0f;
    ef_rate_info.y = 0.0f;
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
    _target_roll_angle = 0.0f;
    _Target_ptr_cam = nullptr;
    _Target_ptr_loc = nullptr;
    _last_ms = millis();
    init_target();
}

void UAttack::udpate_control_value(){
    update_target_pitch_rate();
    update_target_yaw_rate();
    update_target_roll_angle();
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
                                (float)ef_rate_info.x,
                                (float)ef_rate_info.y,
                                (float)_target_pitch_rate,
                                (float)_target_roll_angle,
                                (float)_target_yaw_rate);

    AP::logger().WriteStreaming("UAT2",
                                "TimeUS,angt,angm,agrt,agrm",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)_attack_angle_target,
                                (float)_attack_angle_measure,
                                (float)_attack_angle_rate_target,
                                (float)_attack_angle_rate_measure);

    AP::logger().WriteStreaming("UARL",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_roll_pid.get_pid_info().target,
                                (float)attack_roll_pid.get_pid_info().actual,
                                (float)attack_roll_pid.get_pid_info().FF,
                                (float)attack_roll_pid.get_pid_info().P,
                                (float)attack_roll_pid.get_pid_info().I,
                                (float)attack_roll_pid.get_pid_info().D,
                                (float)attack_roll_pid.get_pid_info().slew_rate,
                                (float)attack_roll_pid.get_pid_info().Dmod);

}

const Vector2f& UAttack::get_bf_info() {
    return bf_info;
}

const Vector2f& UAttack::get_ef_info() {
    return ef_info;
}

const Vector2f& UAttack::get_ef_rate_info() {
    return ef_rate_info;
}


void UAttack::init_target()
{
    bool use_loc = use_target_loc.get();
    bool use_cam = use_target_cam.get();

    if (use_cam) {
        _Target_ptr_cam = new FD_Target_K230();
        if (_Target_ptr_cam->init()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target K230 init");
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target K230 Fail");
            _Target_ptr_cam = nullptr;
        }
    }
    if (use_loc) {
        _Target_ptr_loc = new FD_Target_Loc();
        if (_Target_ptr_loc->init()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc init");
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc Fail");
            _Target_ptr_loc = nullptr;
        }
    }
}

// called at 100 Hz

void UAttack::update()
{
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
        _Target_ptr_loc->update();
    }

    if (_Target_ptr_cam != nullptr && _Target_ptr_cam->is_valid()) {
        if (current_idx != 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to CAM");
        }
        current_idx = 1;
    } else if (_Target_ptr_loc != nullptr && _Target_ptr_loc->is_valid()) {
        if (current_idx != 2) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to LOC");
        }
        current_idx = 2;
    } else {
        if (current_idx != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "No Valid Target");
        }
        current_idx = 0;
    }

    float p1 = 0;
    float p2 = 0;
    if (current_idx == 1) {
        if (_Target_ptr_cam->get_info(p1, p2)) {
            handle_info(p1, p2);
            udpate_control_value();
        }
    } else if (current_idx == 2) {
        if (_Target_ptr_loc->get_info(p1, p2)) {
            handle_info(p1, p2);
            udpate_control_value();
        }
    } else {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
    }

}

void UAttack::handle_info(float p1, float p2) {

    display_info.p3 = p1;
    display_info.p4 = p2;

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();
    // if (!copter.udelay.get_idx(5-1, _roll, _pitch, _yaw)) {
    //     _roll = copter.ahrs_view->roll;
    //     _pitch = copter.ahrs_view->pitch;
    //     _yaw = copter.ahrs_view->yaw;
    // }

    p1 = constrain_float(p1, -80.f, 80.f);
    p2 = constrain_float(p2, -80.f, 80.f);

    bf_info.x = p1; // yaw degree
    bf_info.y = p2; // pitch degree

    float bf_x    =  100.0f;
    float bf_y    =  bf_x*tanf(radians(p1));
    float bf_z    = -bf_x*tanf(radians(p2));
    Vector3f cam_unit = Vector3f(bf_x, bf_y, bf_z);
    cam_unit.normalized();

    Matrix3f tmp_cam_m;
    tmp_cam_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
    Vector3f bf_unit = tmp_cam_m*cam_unit;

    Matrix3f tmp_body_m;
    tmp_body_m.from_euler(_roll, _pitch, _yaw);
    Vector3f ef_unit = tmp_body_m*bf_unit;

    float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.x)));
    float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));

    ef_info.x = angle_yaw;
    ef_info.y = angle_pitch;

    float delta_yaw = wrap_180(wrap_360(angle_yaw) - wrap_360(_last_yaw));
    _last_yaw = angle_yaw;
    _last_yaw_sample += delta_yaw;

    _yaw_sample_filter.apply(_last_yaw_sample);
    _pitch_sample_filter.apply(angle_pitch);

    _yaw_filter.update(_yaw_sample_filter.get(), millis());
    _pitch_filter.update(_pitch_sample_filter.get(), millis());

    ef_rate_info.x = _yaw_filter.slope()*1000.f;
    ef_rate_info.y = _pitch_filter.slope()*1000.f;

    display_info.new_data = true;
    display_info.count++;
}

// degree/second
void UAttack::update_target_pitch_rate() {
    float k1_pitch = attack_k1_pitch.get();
    float k2_pitch = attack_k2_pitch.get();
    float pitch_off = attack_pitch_off.get();
    // float boost_factor = constrain_float(fabsf(bf_info.y)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.y + pitch_off, -30.0f, 30.0f);
    _target_pitch_rate = k1_pitch * ef_rate_info.y + k2_pitch * angle_err; // degrees/s

    //Limit pitch rate
    float limit_pitch_rate = pitch_rate_limit;
    _target_pitch_rate = constrain_float(_target_pitch_rate, -limit_pitch_rate, limit_pitch_rate);

    // //Limit pitch
    // float current_pitch = degrees(plane.ahrs.pitch);
    // float limit_pitch = constrain_float(pitch_limit, -60.f, 60.f);
    // if (current_pitch > limit_pitch) {
    //     _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    // } else if (current_pitch < -limit_pitch) {
    //     _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    // }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree
void UAttack::update_target_roll_angle() {
    // _target_roll_angle = constrain_float(attack_roll_factor.get() * ef_rate_info.x, -15.f, 15.f);
    
    float dt = (millis() - _last_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}
    _target_roll_angle = attack_roll_pid.update_all(0.0f, -ef_rate_info.x, dt);
}

// degree/second
void UAttack::update_target_yaw_rate() {
    float k1_yaw = attack_k1_yaw.get();
    float k2_yaw = attack_k2_yaw.get();
    // float boost_factor = constrain_float(fabsf(bf_info.x)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.x, -30.0f, 30.0f);
    _target_yaw_rate = k1_yaw * ef_rate_info.x + k2_yaw * angle_err;
    display_info.p11 = angle_err;
    display_info.p12 = k2_yaw;
    display_info.p13 = _target_yaw_rate;
    display_info.p14 = plane.uattack.get_target_yaw_rate();
}

void UAttack::handle_attack_msg(const mavlink_message_t &msg) {
    if (_Target_ptr_loc != nullptr) {
        _Target_ptr_loc->handle_msg(msg);
    }
}
