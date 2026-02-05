#include "FD_Target.h"

void FD_Target_Base::handle_info(float p1, float p2) {
    _valid = true;
    _last_ms = millis();
    _p1 = p1;
    _p2 = p2;
    _new_data = true;
}

bool FD_Target_Base::get_info(float &p1, float &p2) {
    if (_new_data) {
        _new_data = false;
        if (_type == 0) {
            p1 = _p1;
            p2 = _p2;
            return true;
        } 
        if (_type == 1) {
            // input in earth frame but yaw with plane
            Vector3f off_ef = Vector3f(1.0f, 0.0f, 0.0f);
            Matrix3f tmp_cam_m;
            tmp_cam_m.from_euler(radians(0.0f), radians(_p2), radians(_p1));
            Matrix3f tmp_earth_m;
            tmp_earth_m.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), radians(0.0f));
            // tmp_earth_m.from_euler(radians(0.0f), radians(0.0f), radians(0.0f));
            tmp_earth_m.transpose();
            Vector3f off_bf = tmp_earth_m*tmp_cam_m*off_ef;
            off_bf.normalized();
            p1 = degrees(wrap_180(atan2f( off_bf.y, off_bf.x))); // x-axis, degrees
            p2 = degrees(wrap_180(atan2f(-off_bf.z, off_bf.xy().length()))); // y-axis, degrees
            return true;
        } 
    }
    return false;
}

void FD_Target_Base::recover_info() {
    _new_data = true;
}

void FD_Target_Base::handle_msg(const mavlink_message_t &msg) {
    return;
}

uint8_t FD_Target_Base::get_type() {
    return _type;
}

void FD_Target_Base::set_type(uint8_t type_in) {
    _type = type_in;
}

void FD_Target_Base::set_valid(bool valid_in) {
    _valid = valid_in;
}

Location& FD_Target_Base::get_target_loc() {
    // _target_loc.lng = 0;
    // _target_loc.lat = 0;
    // _target_loc.alt = 0;
    return _target_loc;
}
