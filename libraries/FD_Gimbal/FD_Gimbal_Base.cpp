#include "FD_Gimbal.h"

bool FD_Gimbal_Base::init() {
    _raw_p1 = 0.0f;
    _raw_p2 = 0.0f;
    return false;
}

void FD_Gimbal_Base::handle_info(float p1, float p2) {
    _valid = true;
    _last_ms = millis();
    _p1 = p1;
    _p2 = p2;
    _new_data = true;
}

void FD_Gimbal_Base::handle_raw_info(float p1, float p2) {
    _raw_p1 = p1;
    _raw_p2 = p2;
}

bool FD_Gimbal_Base::get_info(float &p1, float &p2) {
    if (_new_data) {
        _new_data = false;
        p1 = _p1;
        p2 = _p2;
        return true;
    }
    return false;
}

bool FD_Gimbal_Base::get_raw_info(float &p1, float &p2) {
    p1 = _raw_p1;
    p2 = _raw_p2;
    return true;
}

void FD_Gimbal_Base::handle_msg(const mavlink_message_t &msg) {
    return;
}

void FD_Gimbal_Base::do_rate_control(float pitch_rate, float yaw_rate) {
    return;
}

void FD_Gimbal_Base::get_attitude_euler(float& gimbal_roll, float& gimbal_pitch, float& gimbal_yaw) {
    gimbal_roll = _gimbal_roll;
    gimbal_pitch = _gimbal_pitch;
    gimbal_yaw = _gimbal_yaw;
}

bool FD_Gimbal_Base::have_target() {
    return _valid;
}