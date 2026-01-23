#include "Plane.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_roll_pid    , "ATKRLL_", 0, UAttack, AC_PID),
    AP_GROUPINFO("K1_PTH",      1, UAttack, attack_k1_pitch,         0.0f),
    AP_GROUPINFO("K2_PTH",      2, UAttack, attack_k2_pitch,         1.0f),
    AP_GROUPINFO("K1_YAW",      3, UAttack, attack_k1_yaw,           0.0f),
    AP_GROUPINFO("K2_YAW",      4, UAttack, attack_k2_yaw,           1.0f),
    AP_GROUPINFO("K2_ROLL",     5, UAttack, attack_k2_roll,          0.0f),
    AP_GROUPINFO("K_ANGLE",     6, UAttack, attack_k_angle,          1.0f),
    AP_GROUPINFO("THR",         7, UAttack, attack_throttle,        75.0f),
    AP_GROUPINFO("THR_RATE",    8, UAttack, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("ATKTYPE",     9, UAttack, attack_type,          1000),
    AP_GROUPINFO("ANGLE",      10, UAttack, attack_angle,            0.f),
    AP_GROUPINFO("PTH_LIM",    11, UAttack, pitch_limit,            30.f),
    AP_GROUPINFO("PTH_RLIM",   12, UAttack, pitch_rate_limit,       30.f),
    AP_GROUPINFO("OFF_PTH",    13, UAttack, attack_pitch_off,        0.0f),
    AP_GROUPINFO("UPRINT",     14, UAttack, print,                   0),
    AP_GROUPINFO("TCAM_USE",   15, UAttack, use_target_cam,          0),
    AP_GROUPINFO("TLOC_USE",   16, UAttack, use_target_loc,          0),
    AP_GROUPINFO("TCAM_TYPE",  17, UAttack, use_target_cam_type,     0),
    AP_GROUPINFO("FILT_Y_HZ",  18, UAttack, filt_yaw_hz,             2.0f),
    AP_GROUPINFO("FILT_P_HZ",  19, UAttack, filt_pithc_hz,           2.0f),

    AP_SUBGROUPPTR(_Target_ptr_loc,         "TL_",    20, UAttack,  FD_Target_Loc),
    AP_SUBGROUPPTR(_Target_ptr_cam_DYT,     "TC_",    21, UAttack,  FD_Target_DYT),
    AP_GROUPEND
};

UAttack::UAttack()
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_yaw = 0.0f;
    _last_yaw_sample = 0.0f;
}

// initialise
void UAttack::init()
{
    // udelay.init();
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

    _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
}

void UAttack::udpate_control_value() {
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
                                "TimeUS,type, angt,angm,agrt,agrm",
                                "s-----",
                                "F-----",
                                "Qfffff",
                                AP_HAL::micros64(),
                                (float)get_attack_type(),
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
    bool use_cam = use_target_cam.get();
    bool use_loc = use_target_loc.get();

    if (use_cam) {
         // 1:mav
        if (use_target_cam_type.get() == 1) {
            _Target_ptr_cam_DYT = new FD_Target_DYT();
            if (_Target_ptr_cam_DYT->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target FP847 init");
                _Target_ptr_cam = _Target_ptr_cam_DYT;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam, FD_Target_DYT::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target FP847 Fail");
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
                    Vector3p tmp_off = Vector3p(tmp_vel.x * 2.0f, tmp_vel.y * 2.0f, tmp_vel.z * 2.0f);
                    tmp_loc.offset(tmp_off);
                    _Target_ptr_loc->set_target_loc(tmp_loc);
                }
            }
        }
        _Target_ptr_loc->update();
    }

    if (_Target_ptr_cam != nullptr && _Target_ptr_cam->is_valid()) {
        if (current_idx != 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to CAM");
        }
        current_idx = 1;
        if (_Target_ptr_loc != nullptr && _Target_ptr_loc->is_valid()) {
            _Target_ptr_loc->set_valid(false);
        }
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
            handle_info(p1, p2, _Target_ptr_cam->get_type());
            udpate_control_value();
        }
    } else if (current_idx == 2) {
        if (_Target_ptr_loc->get_info(p1, p2)) {
            handle_info(p1, p2, _Target_ptr_loc->get_type());
            udpate_control_value();
        }
    } else {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
    }

}

void UAttack::handle_info(float p1, float p2, uint8_t cam_type) {

    display_info.p1 = p1;
    display_info.p2 = p2;

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();
    // if (!copter.udelay.get_idx(5-1, _roll, _pitch, _yaw)) {
    //     _roll = copter.ahrs_view->roll;
    //     _pitch = copter.ahrs_view->pitch;
    //     _yaw = copter.ahrs_view->yaw;
    // }
    float angle_pitch = 0.0f;
    float angle_yaw = 0.0f;
    if (cam_type == 0) {
        // body fixed cam
        bf_info.x = p1; // yaw degree
        bf_info.y = p2; // pitch degree

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
    } else {
        // body fixed cam
        bf_info.x = p1; // yaw degree
        bf_info.y = p2; // pitch degree

        // frame with gimbal cam
        angle_pitch = wrap_180(p2);
        angle_yaw =   wrap_180(p1 + degrees(_yaw));
    }

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
    float p = attack_k_angle.get();
    // float boost_factor = constrain_float(fabsf(bf_info.y)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.y + pitch_off, -30.0f, 30.0f);

    _attack_angle_target = attack_angle.get();
    _attack_angle_measure = -ef_info.y;
    _attack_angle_rate_target = (_attack_angle_target - _attack_angle_measure) * p;
    _attack_angle_rate_measure = -ef_rate_info.y;

    float attack_angle_rate_err = _attack_angle_rate_target - _attack_angle_rate_measure;

    attack_angle_rate_err = constrain_float(attack_angle_rate_err, -30.0f, 30.0f);

    _target_pitch_rate = k1_pitch * attack_angle_rate_err + k2_pitch * angle_err; // degrees/s

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

// degree
void UAttack::update_target_roll_angle() {
    // _target_roll_angle = constrain_float(attack_roll_factor.get() * ef_rate_info.x, -15.f, 15.f);
    
    float k2_roll = attack_k2_roll.get();

    float dt = (millis() - _last_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}
    _target_roll_angle = attack_roll_pid.update_all(0.0f, -ef_rate_info.x, dt) + k2_roll * _target_yaw_rate;
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

void UAttack::set_external_cmd(float cmd_speed, float cmd_pitch, float cmd_roll)
{
    _external_cmd._target_speed = cmd_speed;
    _external_cmd._target_pitch = cmd_pitch;
    _external_cmd._target_roll = cmd_roll;
    _external_cmd.last_cmd_ms = millis();
}

uint8_t UAttack::get_attack_type() {
    if ((attack_type.get() == 1) && (millis() - _external_cmd.last_cmd_ms < 1000)) {
        return 1;
    }
    return 0;
}

void UAttack::do_print()
{
    // put your 1Hz code here
    if ((print.get() & (1<<0)) && display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", display_info.count_log, display_info.p1, display_info.p2, display_info.p3, display_info.p4);
        display_info.new_data = false;
    }
    if (print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d", get_ef_info().x,get_ef_info().y, is_active());
    }
    if (print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d", get_ef_rate_info().x,get_ef_rate_info().y, is_active());
    }
    if (print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "ar (%0.1f , %0.1f , %0.2f , %0.2f)", _attack_angle_target, _attack_angle_measure, _attack_angle_rate_target, _attack_angle_rate_measure);
    }
    if (print.get() & (1<<4)) { // 16
        gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f , %0.1f , %0.1f , %0.2f)", get_target_roll_angle(), get_target_pitch_rate(), get_target_yaw_rate(), attack_throttle.get());
    }
    if (print.get() & (1<<5)) { // 32
        gcs().send_text(MAV_SEVERITY_WARNING, "srp (%0.1f , %0.1f , %0.1f)", _external_cmd._target_speed, _external_cmd._target_pitch, _external_cmd._target_roll);
    }
    if (print.get() & (1<<6)) { // 364
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f", display_info.p11, display_info.p12, display_info.p13, display_info.p14);
    }
}