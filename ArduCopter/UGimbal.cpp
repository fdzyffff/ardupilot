#include "Copter.h"

const AP_Param::GroupInfo UGimbal::var_info[] = {

    AP_SUBGROUPINFO(lock_yaw,     "CYAW_", 0, UGimbal, AC_PID),
    AP_SUBGROUPINFO(lock_pitch,   "CPTH_", 1, UGimbal, AC_PID),
    AP_GROUPINFO("TOUT",   2, UGimbal, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  3, UGimbal, cam_width,             360),
    AP_GROUPINFO("PIX_H",  4, UGimbal, cam_height,            360),
    AP_GROUPINFO("ANG_X",  5, UGimbal, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  6, UGimbal, cam_angle_y,           60.0f),

    AP_GROUPEND
};

UGimbal::UGimbal()
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_yaw = 0.0f;
    _last_yaw_sample = 0.0f;
}

// initialise
void UGimbal::init()
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

    _last_control_ms = millis();
    _last_switch_ms = 0;
    _ret_valid = false;

    _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
}

void UGimbal::udpate_control_value(){
    update_gimbal_pitch_rate();
    update_gimbal_yaw_rate();
    _last_control_ms = millis();
    update_log();
}

void UGimbal::update_log() {
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

    AP::logger().WriteStreaming("UATH",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_throttle_pid.get_pid_info().target,
                                (float)attack_throttle_pid.get_pid_info().actual,
                                (float)attack_throttle_pid.get_pid_info().FF,
                                (float)attack_throttle_pid.get_pid_info().P,
                                (float)attack_throttle_pid.get_pid_info().I,
                                (float)attack_throttle_pid.get_pid_info().D,
                                (float)attack_throttle_pid.get_pid_info().slew_rate,
                                (float)attack_throttle_pid.get_pid_info().Dmod);

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

const Vector2f& UGimbal::get_bf_info() {
    return bf_info;
}

const Vector2f& UGimbal::get_ef_info() {
    return ef_info;
}

const Vector2f& UGimbal::get_ef_rate_info() {
    return ef_rate_info;
}


// called at 100 Hz
void UGimbal::update()
{
    uart_gimbal_update();
    uart_ret_update();
    gimbal_control_update();
}

void UGimbal::uart_ret_update()
{
    FD_K230_ptr->read();
    FD_msg_K230 &tmp_msg = FD_K230_ptr->get_msg_K230();
    if (tmp_msg._msg_1.updated) {

        if (tmp_msg._msg_1.content.msg.tag_ok) {
            _last_ret_ms = millis();
            float theta1 = -cal_frame_angle(cam_width.get(), cam_angle_x.get(), tmp_msg._msg_1.content.msg.tag_x); // x-axis, degree
            float theta2 =  cal_frame_angle(cam_height.get(), cam_angle_y.get(), tmp_msg._msg_1.content.msg.tag_y); // y-axis, degree

            Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
            float p1 = degrees(atanf(tmp.y/tmp.x));
            float p2 = degrees(atanf(tmp.z/tmp.xy().length()));
            cal_and_handle(p1, p2);
        }

        tmp_msg._msg_1.updated = false;   
    }
    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ret_ms的值，如果tnow赋值在其之前，则会小于_last_ret_ms。SITL仿不出来，它周期是50Hz太低了
    if ((target_timeout > 0) && (tnow - _last_ret_ms > (uint32_t)target_timeout)) {
        // if (_ret_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "valid %ld|%ld", tnow, _last_ret_ms);
        // }
        _ret_valid = false;
    }
}

void UGimbal::cal_and_handle(float p1, float p2) 
{
    AP_Mount* mount = AP::mount();
    float cam_roll, cam_pitch, cam_bf_yaw;
    if (mount != nullptr) {
        mount->get_attitude_euler(0, cam_roll, cam_pitch, cam_bf_yaw);
    }

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();

    Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_level_m;
    tmp_cam_level_m.from_euler(radians(cam_roll), radians(cam_pitch), radians(cam_bf_yaw));
    Matrix3f tmp_level_body_m;
    tmp_level_body_m.from_euler(_roll, _pitch, 0.0f);
    tmp_level_body_m.transpose();
    Matrix3f tmp_target_earth_m = tmp_level_body_m*tmp_cam_level_m*tmp_target_cam_m;
    Vector3f bf_unit = tmp_target_earth_m*target_unit;

    float angle_yaw =   wrap_180(degrees(atan2f( bf_unit.y, bf_unit.x)));
    float angle_pitch = wrap_180(degrees(atan2f(-bf_unit.z, bf_unit.xy().length())));
    
    _last_ret_ms = millis();
    _p1 = angle_yaw;
    _p2 = angle_pitch;
    _new_data = true;
    _ret_valid = true;
}


void UGimbal::gimbal_control_update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
        //update filter cutoff HZ in flight
        _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
        _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    }

    float p1 = _p1;
    float p2 = _p2;
    if (_ret_valid) {
        if (_new_data) {
            handle_info_final(p1, p2);
            udpate_control_value();
            _new_data = false;
        }
        _state = Gimbal_State::AHead;
    } else {
        _state = Gimbal_State::Search;
    }

    switch (_state) {
        case Gimbal_State::AHead:
        {
            float target_pitch = 0.0f;
            float target_yaw = 0.0f;
        }
        break;
        case Gimbal_State::Search:
        {
            float target_pitch = 0.0f;
            float target_yaw = 0.0f;
        }
        break;
        case Gimbal_State::Lock:
        {
            if (_ret_valid) {
                if (_new_data) {
                    handle_info_final(p1, p2);
                    float target_pitch = 0.0f;
                    float target_yaw = 0.0f;
                }
            } else {
                set_state(Gimbal_State::Search);
            }
        }
        break;
    }
    update_gimbal_control();
}

void UGimbal::handle_info_final(float p1, float p2) {

    display_info.p3 = p1;
    display_info.p4 = p2;

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();
    // if (!udelay.get_idx(10-1, _roll, _pitch, _yaw)) {
    //     _roll = AP::ahrs().get_roll();
    //     _pitch = AP::ahrs().get_pitch();
    //     _yaw = AP::ahrs().get_yaw();
    // }

    bf_info.x = p1; // yaw degree
    bf_info.y = p2; // pitch degree

    if (p2 < -90.f) {
        p2 = -180.0f - p2;
    } else if (p2 > 90.0f) {
        p2 = 180.0f - p2;
    }

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

    float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
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
void UGimbal::update_gimbal_pitch_rate() {
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
    // float current_pitch = degrees(copter.ahrs.pitch);
    // float limit_pitch = constrain_float(pitch_limit, -60.f, 60.f);
    // if (current_pitch > limit_pitch) {
    //     _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    // } else if (current_pitch < -limit_pitch) {
    //     _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    // }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree
void UGimbal::update_target_roll_angle() {
    // _target_roll_angle = constrain_float(attack_roll_factor.get() * ef_rate_info.x, -15.f, 15.f);
    float k2_roll = attack_k2_roll.get();

    float dt = (millis() - _last_control_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}
    _target_roll_angle = attack_roll_pid.update_all(0.0f, -ef_rate_info.x, dt) + k2_roll * _target_yaw_rate;
}

// degree/second
void UGimbal::update_gimbal_yaw_rate() {
    float k1_yaw = attack_k1_yaw.get();
    float k2_yaw = attack_k2_yaw.get();
    // float boost_factor = constrain_float(fabsf(bf_info.x)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.x, -30.0f, 30.0f);
    _target_yaw_rate = k1_yaw * ef_rate_info.x + k2_yaw * angle_err;
    display_info.p11 = angle_err;
    display_info.p12 = k2_yaw;
    display_info.p13 = _target_yaw_rate;
    display_info.p14 = copter.UGimbal.get_target_yaw_rate();
}


void UGimbal::handle_attack_msg(const mavlink_message_t &msg) {
    if (_Target_ptr_loc != nullptr) {
        _Target_ptr_loc->handle_msg(msg);
    }
    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->handle_msg(msg);
    }
}


// UDelay
void UGimbal::UDelay::init()
{
    _idx = 0;
    for (uint16_t i = 0; i < UDELAY_BUFFER; i++) {
        _buffer[i].roll = 0.0f;
        _buffer[i].pitch = 0.0f;
        _buffer[i].yaw = 0.0f;
        _buffer[i].time_ms = 0;
    }
}

void UGimbal::UDelay::push()
{
    _idx += 1;
    if (_idx >= UDELAY_BUFFER) {
        _idx = 0;
    }
    _buffer[_idx].roll  = AP::ahrs().get_roll();
    _buffer[_idx].pitch = AP::ahrs().get_pitch();
    _buffer[_idx].yaw   = AP::ahrs().get_yaw();
    _buffer[_idx].time_ms = millis();
}

bool UGimbal::UDelay::get_idx(uint16_t step, float &roll, float &pitch, float &yaw) 
{
    uint16_t this_idx = 0;
    if (_idx >= step) {
        this_idx = _idx - step;
    } else {
        this_idx = UDELAY_BUFFER + _idx - step;
    }
    roll = _buffer[this_idx].roll;
    pitch = _buffer[this_idx].pitch;
    yaw = _buffer[this_idx].yaw;
    if (millis() - _buffer[this_idx].time_ms > 500) {
        return false;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", (millis()-_buffer[this_idx].time_ms));
    return true;
}
