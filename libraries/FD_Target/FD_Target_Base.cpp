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
        p1 = _p1;
        p2 = _p2;
        return true;
    }
    return false;
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
