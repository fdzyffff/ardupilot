#include "Plane.h"
//#include "UCam.h"

UAttack::UAttack()
{
    ;
}

// initialise
void UAttack::init()
{
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
    _target_type = 0;
    _UTarget_ptr = nullptr;
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
                                (float)plane.g2.attack_roll_pid.get_pid_info().target,
                                (float)plane.g2.attack_roll_pid.get_pid_info().actual,
                                (float)plane.g2.attack_roll_pid.get_pid_info().FF,
                                (float)plane.g2.attack_roll_pid.get_pid_info().P,
                                (float)plane.g2.attack_roll_pid.get_pid_info().I,
                                (float)plane.g2.attack_roll_pid.get_pid_info().D,
                                (float)plane.g2.attack_roll_pid.get_pid_info().slew_rate,
                                (float)plane.g2.attack_roll_pid.get_pid_info().Dmod);

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

void UAttack::delete_target() {
    if (_UTarget_ptr != nullptr) {
        delete _UTarget_ptr;
        _UTarget_ptr = nullptr;
        _target_type = 0;
        gcs().send_text(MAV_SEVERITY_WARNING, "Del Target Instance");
    }
}

void UAttack::init_target()
{
    uint8_t param_target_type = plane.g2.user_target_type.get();

    if (_target_type != param_target_type) {
        delete_target();

        if (param_target_type == 1) {
            _UTarget_ptr = new UTarget_Cam(*this);
            if (_UTarget_ptr->init()) {
                _target_type = param_target_type; 
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Cam init");
            } else {
                delete_target();
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Cam Fail");
            }
        }

        if (param_target_type == 2) {
            _UTarget_ptr = new UTarget_Loc(*this);
            if (_UTarget_ptr->init()) {
                _target_type = param_target_type; 
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc init");
            } else {
                delete_target();
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Loc Fail");
            }
        }

        if (param_target_type == 3) {
            _UTarget_ptr = new UTarget_Mav(*this);
            if (_UTarget_ptr->init()) {
                _target_type = param_target_type; 
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Mav init");
            } else {
                delete_target();
                gcs().send_text(MAV_SEVERITY_WARNING, "Target Mav Fail");
            }
        }
    }
}

// called at 100 Hz
void UAttack::target_update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
    }

    if (_UTarget_ptr == nullptr) {return;}
    _UTarget_ptr->update();
}

// update
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

    if (_UTarget_ptr!= nullptr) {
        _UTarget_ptr->update();
    }
    time_out_check();
}

void UAttack::time_out_check() {
    if (_UTarget_ptr == nullptr) {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
        _active = false;
        return;
    }
    if (!_UTarget_ptr->is_valid()) { 
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
        if (_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Lost Target");
        }
        _active = false;
        return;
    } else {
        if (!_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Got Target");
        }
        _active = true;
    }
}


// degree/second
void UAttack::update_target_pitch_rate() {
    float k1_pitch = plane.g2.attack_k1_pitch.get();
    float k2_pitch = plane.g2.attack_k2_pitch.get();
    float pitch_off = plane.g2.attack_pitch_off.get();
    // float boost_factor = constrain_float(fabsf(bf_info.y)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.y + pitch_off, -30.0f, 30.0f);
    _target_pitch_rate = k1_pitch * ef_rate_info.y + k2_pitch * angle_err; // degrees/s

    //Limit pitch rate
    float limit_pitch_rate = plane.g2.pitch_rate_limit;
    _target_pitch_rate = constrain_float(_target_pitch_rate, -limit_pitch_rate, limit_pitch_rate);

    // //Limit pitch
    // float current_pitch = degrees(plane.ahrs.pitch);
    // float limit_pitch = constrain_float(plane.g2.pitch_limit, -60.f, 60.f);
    // if (current_pitch > limit_pitch) {
    //     _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    // } else if (current_pitch < -limit_pitch) {
    //     _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    // }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree
void UAttack::update_target_roll_angle() {
    // _target_roll_angle = constrain_float(plane.g2.attack_roll_factor.get() * ef_rate_info.x, -15.f, 15.f);
    
    float dt = (millis() - _last_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}
    _target_roll_angle = plane.g2.attack_roll_pid.update_all(0.0f, -ef_rate_info.x, dt);
}

// degree/second
void UAttack::update_target_yaw_rate() {
    float k1_yaw = plane.g2.attack_k1_yaw.get();
    float k2_yaw = plane.g2.attack_k2_yaw.get();
    // float boost_factor = constrain_float(fabsf(bf_info.x)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.x, -30.0f, 30.0f);
    _target_yaw_rate = k1_yaw * ef_rate_info.x + k2_yaw * angle_err;
    display_info.p11 = angle_err;
    display_info.p12 = k2_yaw;
    display_info.p13 = _target_yaw_rate;
    display_info.p14 = plane.uattack.get_target_yaw_rate();
}

void UAttack::handle_attack_msg(const mavlink_message_t &msg) {
    if ((_target_type == 2 || _target_type == 3) && _UTarget_ptr != nullptr) {
        _UTarget_ptr->handle_msg(msg);
    }
}
