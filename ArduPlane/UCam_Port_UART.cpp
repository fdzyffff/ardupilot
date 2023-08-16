#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UCam_Port_UART::UCam_Port_UART(UCam &frotend_in, AP_HAL::UARTDriver* port_in):
    UCam_Port(frotend_in)
{
    FD1_uart_ptr = new FD1_uart_msg(port_in);
    FD1_uart_ptr->FD1_msg_DYT().set_enable();
}

void UCam_Port_UART::port_read() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();

    FD1_msg_DYT &tmp_msg = FD1_uart_ptr->FD1_msg_DYT();
    if (tmp_msg._msg_1.updated) {
        handle_info();
    }
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

void UCam_Port_UART::term_clear() {
    memset(&_term, 0, sizeof(_term));
}

bool UCam_Port_UART::detect_sentence() {
    if (strcmp(_term, "result0") == 0) {
        return true;
    }
    return false;
}

float UCam_Port_UART::get_number() {
    char *endptr = nullptr;
    float x = strtod(_term, &endptr);
    return x;
}

void UCam_Port_UART::fill_number()
{
    switch (_term_number) {
        case 1: // label
            break;
        case 2: // left
            _left = get_number();
            break;
        case 3: // right
            _right = get_number();
            break;
        case 4: // top
            _top = get_number();
            break;
        case 5: // bottom
            _bottom = get_number();
            break;
        default:
            break;
    }
}

void UCam_Port_UART::handle_info() {
    _frotend.display_info_p1 = _left;
    _frotend.display_info_p2 = _right;
    _frotend.display_info_p3 = _top;
    _frotend.display_info_p4 = _bottom;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
    bool _valid = false;
    _frotend._cam_state = 5;
    _valid = true;

    if (!_valid) {
        _frotend._n_count = 0;
        return;
    }

    float dt = (float)(millis() - _frotend._last_update_ms)*1.0e-3f;
    if (dt > 1.0f) {dt = 1.0f;}
    if (dt < 0.01f) {dt = 0.01f;} // sainty check, cam info will be at around 20Hz, conrresponding to 0.05s.
    
    float p1 = (_left + _right) * 0.5f;
    float p2 = (_top + _bottom) * 0.5f;

    _frotend.raw_info.x = p1;
    _frotend.raw_info.y = p2;
    Matrix3f tmp_m;
    if (is_zero(copter.g2.user_parameters.fly_roll_factor)) {
        tmp_m.from_euler(0.0f, 0.0f, 0.0f);
    } else {
        tmp_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    }
    Vector3f tmp_input = Vector3f(100.f,p1,-p2);
    Vector3f tmp_output = tmp_m*tmp_input;
    _frotend._cam_filter.apply(tmp_output, dt);
    _frotend.correct_info.x = _frotend._cam_filter.get().y;
    _frotend.correct_info.y = -_frotend._cam_filter.get().z;
    _frotend._last_update_ms = millis();
    if (!_frotend._active) {
        _frotend._n_count += 1;
    }
    _frotend.udpate_value(dt);
}

void UCam_Port_UART::do_cmd() {
    ;
}
