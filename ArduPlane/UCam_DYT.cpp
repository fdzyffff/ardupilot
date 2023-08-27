#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UCam_DYT::UCam_DYT(UAttack &frotend_in, AP_HAL::UARTDriver* port_in):
    UCam_base(frotend_in)
{
    FD1_uart_ptr = new FD1_UART(port_in);
    FD1_uart_ptr->get_msg_DYT().set_enable();
    _yaw_rate_filter.set_cutoff_frequency(100.f, 20.f);
    return;
}

void UCam_DYT::update() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    _yaw_rate_filter.apply(degrees(plane.ahrs.get_yaw_rate_earth()));

    FD1_uart_ptr->read();
    FD1_msg_DYT &tmp_msg = FD1_uart_ptr->get_msg_DYT();
    if (tmp_msg._msg_1.updated) {
        _valid = true;
        _last_ms = millis();
        handle_info();
    }

    if (tnow - _last_ms > 1000) {
        _valid = false;
        _pitch_filter.reset();
        _yaw_filter.reset();
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

void UCam_DYT::do_cmd() {
    ;
}

bool UCam_DYT::is_valid() {
    return _valid;
}

void UCam_DYT::handle_info() {
    // FD1_msg_DYT &tmp_msg = FD1_uart_ptr->get_msg_DYT();
    float p1 = 0.0f; // x-axis
    float p2 = 0.0f; // y-axis

    _frotend.raw_info.x = p1;
    _frotend.raw_info.y = p2;

    Matrix3f tmp_m;
    tmp_m.from_euler(plane.ahrs.roll, plane.ahrs.pitch, 0.0f);

    float dist_z = -tanf(p1);
    float dist_y = tanf(p2);

    Vector3f tmp_input = Vector3f(1.0f,dist_y,dist_z);
    Vector3f tmp_output = tmp_m*tmp_input;

    float angle_pitch = atan2f(-tmp_output.z, tmp_output.x);
    float angle_yaw = atan2f(tmp_output.y, tmp_output.x);

    _pitch_filter.update(angle_pitch, millis());
    _yaw_filter.update(angle_yaw, millis());

    _frotend.correct_info.x = _yaw_rate_filter.get() + _yaw_filter.slope()*1000.f;
    _frotend.correct_info.y = _pitch_filter.slope()*1000.f;

    _frotend.udpate_control_value();

    _frotend.display_info_p1 = p1;
    _frotend.display_info_p2 = p2;
    _frotend.display_info_p3 = 0.0f;
    _frotend.display_info_p4 = 0.0f;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
}
