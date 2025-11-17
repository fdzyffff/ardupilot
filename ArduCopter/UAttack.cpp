#include "Copter.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_roll_pid    , "ATKRLL_", 0, UAttack, AC_PID),
    AP_GROUPINFO("K1_PTH",      1, UAttack, attack_k1_pitch,         1.0f),
    AP_GROUPINFO("K2_PTH",      2, UAttack, attack_k2_pitch,         1.0f),
    AP_GROUPINFO("K1_YAW",      3, UAttack, attack_k1_yaw,           0.0f),
    AP_GROUPINFO("K2_YAW",      4, UAttack, attack_k2_yaw,           1.0f),
    AP_GROUPINFO("K2_ROLL",     5, UAttack, attack_k2_roll,          0.0f),
    AP_GROUPINFO("K_ANGLE",     6, UAttack, attack_k_angle,          1.0f),
    AP_GROUPINFO("THR",         7, UAttack, attack_throttle,        75.0f),
    AP_GROUPINFO("THR_RATE",    8, UAttack, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("OUTMS",       9, UAttack, attack_timeout,       2000),
    AP_GROUPINFO("ANGLE",      10, UAttack, attack_angle,           30.f),
    AP_GROUPINFO("PTH_LIM",    11, UAttack, pitch_limit,            30.f),
    AP_GROUPINFO("PTH_RLIM",   12, UAttack, pitch_rate_limit,       30.f),
    AP_GROUPINFO("OFF_PTH",    13, UAttack, attack_pitch_off,        0.0f),
    AP_GROUPINFO("UPRINT",     14, UAttack, print,                   0),
    AP_GROUPINFO("TCAM_USE",   15, UAttack, use_target_cam,          0),
    AP_GROUPINFO("TLOC_USE",   16, UAttack, use_target_loc,          0),
    AP_GROUPINFO("TCAM_TYPE",  17, UAttack, use_target_cam_type,     0),
    AP_GROUPINFO("FILT_Y_HZ",  18, UAttack, filt_yaw_hz,             2.0f),
    AP_GROUPINFO("FILT_P_HZ",  19, UAttack, filt_pithc_hz,           2.0f),
    AP_GROUPINFO("V_TARGET",   20, UAttack, attack_vel,              15.0f),
    AP_GROUPINFO("V_MAX_XY",   21, UAttack, max_vel_xy,              15.0f),
    AP_GROUPINFO("V_MAX_Z",    22, UAttack, max_vel_z,               5.0f),

    AP_SUBGROUPPTR(_Target_ptr_loc,           "TL_",    23, UAttack,  FD_Target_Loc),
    AP_SUBGROUPPTR(_Target_ptr_cam_mav,       "TC0_",   24, UAttack,  FD_Target_Mav),
    AP_SUBGROUPPTR(_Target_ptr_cam_rk3588,    "TC1_",   25, UAttack,  FD_Target_RK3588),
    AP_SUBGROUPPTR(_Target_ptr_cam_k230,      "TC2_",   26, UAttack,  FD_Target_K230),
    AP_SUBGROUPPTR(_Target_ptr_cam_topotek,   "TC5_",   27, UAttack,  FD_Target_Topotek),
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
    _target_yaw_rate = 0.0f;
    _Target_ptr_cam = nullptr;
    _Target_ptr_loc = nullptr;
    _last_ms = millis();
    init_target();

    _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
}

void UAttack::udpate_control_value(){
    update_target_yaw_rate();
    _last_ms = millis();
    update_log();
}

void UAttack::update_log() {
    AP::logger().WriteStreaming("UATK",
                                "TimeUS,bfx,bfy,efx,efy,efrx,efry,tyaw",
                                "s-------",
                                "F-------",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                (float)bf_info.x,
                                (float)bf_info.y,
                                (float)ef_info.x,
                                (float)ef_info.y,
                                (float)ef_rate_info.x,
                                (float)ef_rate_info.y,
                                (float)_target_yaw_rate);

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
         // 1:mav, 2:RK3588, 3:K230, 4:LRB
        if (use_target_cam_type.get() == 1) {
            _Target_ptr_cam_mav = new FD_Target_Mav();
            if (_Target_ptr_cam_mav->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Mav init");
                _Target_ptr_cam = _Target_ptr_cam_mav;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam, FD_Target_Mav::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Mav Fail");
                _Target_ptr_cam_mav = nullptr;
            }
        } 
        else if (use_target_cam_type.get() == 2) {
            _Target_ptr_cam_rk3588 = new FD_Target_RK3588();
            if (_Target_ptr_cam_rk3588->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target RK3588 init");
                _Target_ptr_cam = _Target_ptr_cam_rk3588;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam_rk3588, FD_Target_RK3588::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target RK3588 Fail");
                _Target_ptr_cam_rk3588 = nullptr;
            }
        } 
        else if (use_target_cam_type.get() == 3) {
            _Target_ptr_cam_k230= new FD_Target_K230();
            if (_Target_ptr_cam_k230->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target K230 init");
                _Target_ptr_cam = _Target_ptr_cam_k230;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam_k230, FD_Target_K230::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target K230 Fail");
                _Target_ptr_cam_k230= nullptr;
            }
        } 
        else if (use_target_cam_type.get() == 4) {
            _Target_ptr_cam_lrb = new FD_Target_LRB();
            if (_Target_ptr_cam_lrb->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target LRB init");
                _Target_ptr_cam = _Target_ptr_cam_lrb;
                // AP_Param::load_object_from_eeprom(_Target_ptr_cam_lrb, FD_Target_LRB::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target LRB Fail");
                _Target_ptr_cam_lrb = nullptr;
            }
        } 
        else if (use_target_cam_type.get() == 5) {
            _Target_ptr_cam_topotek = new FD_Target_Topotek();
            if (_Target_ptr_cam_topotek->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Topotek init");
                _Target_ptr_cam = _Target_ptr_cam_topotek;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam_topotek, FD_Target_Topotek::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Topotek Fail");
                _Target_ptr_cam_topotek = nullptr;
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
            if (_Target_ptr_cam->get_type() == 0) {
                handle_info_bodycam(p1, p2);
            } else {
                handle_info_gimbal(p1, p2);
            }
            udpate_control_value();
        }
    } else if (current_idx == 2) {
        if (_Target_ptr_loc->get_info(p1, p2)) {
            handle_info_gimbal(p1, p2);
            udpate_control_value();
        }
    } else {
        _target_yaw_rate = 0.0f;
    }

}

// body fixed cam
void UAttack::handle_info_bodycam(float p1, float p2) {

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

    bf_info.x = p1; // yaw degree
    bf_info.y = p2; // pitch degree

    if (p2 < -90.f) {
        p2 = -180.0f - p2;
    } else if (p2 > 90.0f) {
        p2 = 180.0f - p2;
    }

    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_body_m;
    tmp_cam_body_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
    Matrix3f tmp_body_earth_m;
    tmp_body_earth_m.from_euler(_roll, _pitch, _yaw);
    Matrix3f tmp_target_earth_m = tmp_body_earth_m*tmp_cam_body_m*tmp_target_cam_m;

    float tmp_roll = 0.0f;
    float tmp_pitch = 0.0f;
    float tmp_yaw = 0.0f;

    tmp_target_earth_m.to_euler(&tmp_roll, &tmp_pitch, &tmp_yaw);

    float angle_pitch = wrap_180(degrees(tmp_pitch));
    float angle_yaw =   wrap_180(degrees(tmp_yaw));

    // Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
    // Matrix3f tmp_target_cam_m;
    // tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    // Vector3f cam_unit = tmp_target_cam_m*target_unit;

    // Matrix3f tmp_cam_body_m;
    // tmp_cam_body_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
    // Vector3f bf_unit = tmp_cam_body_m*cam_unit;

    // Matrix3f tmp_body_earth_m;
    // tmp_body_earth_m.from_euler(_roll, _pitch, _yaw);
    // Vector3f ef_unit = tmp_body_earth_m*bf_unit;

    // float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
    // float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));

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

// frame with gimbal cam
void UAttack::handle_info_gimbal(float p1, float p2) {

    display_info.p3 = p1;
    display_info.p4 = p2;

    float _yaw = AP::ahrs().get_yaw();

    float angle_pitch = wrap_180(p2);
    float angle_yaw =   wrap_180(p1 + degrees(_yaw));

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
