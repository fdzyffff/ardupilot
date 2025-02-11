#include "Copter.h"
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
    display_info_new = false;
    display_info_p1 = 0.0f;
    display_info_p2 = 0.0f;
    display_info_p3 = 0.0f;
    display_info_p4 = 0.0f;
    display_info_count = 0;
    display_info_count_log = 0;
    _target_pitch_rate = 0.0f;
    _target_yaw_rate = 0.0f;
    _target_roll_angle = 0.0f;
    _cam_port_type = 0;
    _UCam_ptr = nullptr;
    _last_ms = millis();
    init_cam_port();
}

void UAttack::udpate_control_value(){
    update_target_pitch_rate();
    update_target_yaw_rate();
    update_target_roll_angle();
    update_target_throttle();
    update_target_bf_to_ef();
    _last_ms = millis();
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


void UAttack::init_cam_port()
{
    if (_cam_port_type != 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "CAM already init");
        return;
    }
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _cam_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CAM, 0);
    if (_cam_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "CAM init");
        _UCam_ptr = new UCam(*this, _cam_port);
        _cam_port_type = 1;
        return;
    }

    _cam_port_type = 0;
    gcs().send_text(MAV_SEVERITY_WARNING, "CAM init FAILED");
}

// called at 100 Hz
void UAttack::cam_update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info_count_log = display_info_count;
        display_info_count = 0;
        last_count_ms = tnow_ms;
    }

    if (get_port() == NULL) {return;}
    if (_UCam_ptr == nullptr) {return;}
    _UCam_ptr->update();
}


void UAttack::do_cmd_on(bool on)
{
    if (get_port() == NULL) {return;}
    if (_UCam_ptr == nullptr) {return;}
    _UCam_ptr->do_cmd_on(on);
}


// update
void UAttack::update()
{
    cam_update();
    time_out_check();
}

void UAttack::time_out_check() {
    if (_UCam_ptr == nullptr) {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
        _active = false;
        return;
    }
    if (!_UCam_ptr->is_valid()) { 
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
        _attack_throttle = copter.motors->get_throttle_hover();
        if (_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Lost CAM");
        }
        _active = false;
        return;
    } else {
        if (!_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Got CAM");
        }
        _active = true;
    }
}

// degree/second
void UAttack::update_target_pitch_rate() {
    float k1_pitch = copter.g2.user_parameters.attack_k1_pitch.get();
    float k2_pitch = copter.g2.user_parameters.attack_k2_pitch.get();
    // float boost_factor = constrain_float(fabsf(bf_info.y)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.y, -30.0f, 30.0f);
    _target_pitch_rate = k1_pitch * ef_rate_info.y + k2_pitch * angle_err; // degrees/s

    //Limit pitch rate
    float limit_pitch_rate = copter.g2.user_parameters.pitch_rate_limit;
    _target_pitch_rate = constrain_float(_target_pitch_rate, -limit_pitch_rate, limit_pitch_rate);

    // //Limit pitch
    // float current_pitch = degrees(copter.ahrs.pitch);
    // float limit_pitch = constrain_float(copter.g2.user_parameters.pitch_limit, -60.f, 60.f);
    // if (current_pitch > limit_pitch) {
    //     _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    // } else if (current_pitch < -limit_pitch) {
    //     _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    // }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree
void UAttack::update_target_roll_angle() {
    _target_roll_angle = constrain_float(copter.g2.user_parameters.attack_roll_factor.get() * ef_rate_info.x, -15.f, 15.f);
}

// degree/second
void UAttack::update_target_yaw_rate() {
    float k1_yaw = copter.g2.user_parameters.attack_k1_yaw.get();
    float k2_yaw = copter.g2.user_parameters.attack_k2_yaw.get();
    // float boost_factor = constrain_float(fabsf(bf_info.x)/15.0f, 0.0f, 1.0f) * 2.0f;
    float angle_err = constrain_float(bf_info.x, -30.0f, 30.0f);
    _target_yaw_rate = k1_yaw * ef_rate_info.x + k2_yaw * angle_err;
}

// from 0 to 1, according to ef_info.y, the pitch angle of body-target in earth frame
void UAttack::update_target_throttle() {
    float p = copter.g2.user_parameters.attack_k_angle.get();
    _attack_angle_target = copter.g2.user_parameters.attack_angle.get();
    _attack_angle_measure = -ef_info.y;
    _attack_angle_rate_target = (_attack_angle_target - _attack_angle_measure) /45.0f * p;
    _attack_angle_rate_measure = -ef_rate_info.y;

    float dt = (millis() - _last_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {dt = 0.2f;}
    _attack_throttle = copter.g2.user_parameters.attack_throttle_pid.update_all(_attack_angle_rate_target, _attack_angle_rate_measure, dt);

    _attack_throttle_p = copter.g2.user_parameters.attack_throttle_pid.get_p();
    _attack_throttle_i = copter.g2.user_parameters.attack_throttle_pid.get_i();
    _attack_throttle_d = copter.g2.user_parameters.attack_throttle_pid.get_d();
    _attack_throttle_pid = _attack_throttle_p + _attack_throttle_i + _attack_throttle_d;
}

// // from 0 to 1, according to ef_info.y, the pitch angle of body-target in earth frame
// void UAttack::update_target_throttle() {
//     float p = copter.g2.user_parameters.attack_angle_kp.get();
//     _attack_angle_target = copter.g2.user_parameters.attack_angle.get();
//     _attack_angle_measure = -degrees(copter.ahrs_view->pitch);
//     _attack_angle_rate_target = (_attack_angle_target - _attack_angle_measure) /45.0f * p;
//     _attack_angle_rate_measure = -ef_rate_info.y;

//     float dt = (millis() - _last_ms);
//     dt = dt * 0.001f;
//     if (dt > 0.2f) {dt = 0.2f;}
//     _attack_throttle = copter.g2.user_parameters.attack_throttle_pid.update_all(_attack_angle_rate_target, _attack_angle_rate_measure, dt);

//     _attack_throttle_p = copter.g2.user_parameters.attack_throttle_pid.get_p();
//     _attack_throttle_i = copter.g2.user_parameters.attack_throttle_pid.get_i();
//     _attack_throttle_d = copter.g2.user_parameters.attack_throttle_pid.get_d();
//     _attack_throttle_pid = _attack_throttle_p + _attack_throttle_i + _attack_throttle_d;
// }

void UAttack::update_target_bf_to_ef() {
    Matrix3f tmp_bf_m;
    tmp_bf_m.from_euler(0.0f, _target_pitch_rate, _target_yaw_rate);
    Matrix3f tmp_roll_m;
    tmp_roll_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    Matrix3f tmp_ef_m = tmp_roll_m*tmp_bf_m;

    float tmp_roll_rate;
    float tmp_pitch_rate;
    float tmp_yaw_rate;
    tmp_ef_m.to_euler(&tmp_roll_rate, &tmp_pitch_rate, &tmp_yaw_rate);

    _target_pitch_rate = tmp_pitch_rate;
    _target_yaw_rate = tmp_yaw_rate;
}

void UAttack::handle_info_test(float p1, float p2) {
    if (_cam_port_type == 1 || _cam_port_type == 2) {
        _UCam_ptr->handle_info_test(p1, p2);
    }
}
