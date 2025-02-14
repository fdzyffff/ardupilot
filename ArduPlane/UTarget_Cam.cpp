#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UTarget_Cam::UTarget_Cam(UAttack &frotend_in):
    UTarget_Base(frotend_in)
{
    _yaw_rate_filter.set_cutoff_frequency(10.f, 25.f);
    return;
}

bool UTarget_Cam::init() {

    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    // _target_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CamDYT, 0);

    // if (_target_port != nullptr) {
    //     FD1_uart_ptr = new FD_DYT_NEW(port_in);
    //     FD1_uart_ptr->get_msg_DYTTARGET().set_enable();
    //     return true;
    // }

    return false;
}

void UTarget_Cam::update() {
    // static uint32_t last_update_ms = millis();
    // _yaw_rate_filter.apply(degrees(plane.ahrs.get_yaw_rate_earth()));

    // FD1_uart_ptr->read();
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // if (tmp_msg._msg_1.updated) {
    //     // DYT -> APM
    //     if (tmp_msg._msg_1.content.msg.target_x == 0 && tmp_msg._msg_1.content.msg.target_y == 0) {
    //         // unhealthy massage
    //         ;
    //     } else {
    //         float p1 = (float)(tmp_msg._msg_1.content.msg.target_x) * 0.05f; // x-axis
    //         float p2 = (float)(tmp_msg._msg_1.content.msg.target_y) * 0.05f; // y-axis
    //         handle_info(p1, p2);
    //     }
    //     tmp_msg._msg_1.updated = false;
    // }

    // uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    // if ((plane.g2.user_attack_timeout > 0) && (tnow - _last_ms > (uint32_t)plane.g2.user_attack_timeout)) {
    //     // if (_valid) {
    //     //     gcs().send_text(MAV_SEVERITY_INFO, "valid %ld|%ld", tnow, _last_ms);
    //     // }
    //     _valid = false;
    //     _pitch_filter.reset();
    //     _yaw_filter.reset();
    // }

    // // for print purpose
    // if (tnow - last_update_ms > 1000) {
    //     //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
    //     last_update_ms = tnow;
    // }

    // fill_state_msg(); // APM -> DYT
    // foward_DYT_mavlink(); // DYT -> APM -> GCS
}

void UTarget_Cam::do_cmd() {
    ;
}

bool UTarget_Cam::is_valid() {
    return _valid;
}

void UTarget_Cam::handle_info(float p1, float p2) {
    //     // if (!_valid) {
    //     //     gcs().send_text(MAV_SEVERITY_INFO, "IIvalid %ld|%ld", millis(), _last_ms);
    //     // }
    // _valid = true;
    // _last_ms = millis();

    // float _roll = plane.ahrs.roll;
    // float _pitch = plane.ahrs.pitch;
    // float _yaw = plane.ahrs.yaw;
    // if (!plane.udelay.get_idx(24-1, _roll, _pitch, _yaw)) {
    //     _roll = plane.ahrs.roll;
    //     _pitch = plane.ahrs.pitch;
    //     _yaw = plane.ahrs.yaw;
    // }

    // _frotend.bf_info.x = p1; // yaw degree
    // _frotend.bf_info.y = p2; // pitch degree

    // Matrix3f tmp_m;
    // tmp_m.from_euler(_roll, _pitch, 0.0f);

    // float dist_z = -tanf(radians(p2));
    // float dist_y = tanf(radians(p1));

    // Vector3f tmp_input = Vector3f(1.0f,dist_y,dist_z);
    // Vector3f tmp_output = tmp_m*tmp_input;

    // float angle_pitch = wrap_180(degrees(atan2f(-tmp_output.z, tmp_output.x)));
    // float angle_yaw = wrap_180(degrees(atan2f(tmp_output.y, tmp_output.x)));

    // _frotend.ef_info.x = wrap_360(angle_yaw + degrees(_yaw));
    // _frotend.ef_info.y = angle_pitch;

    // _yaw_filter.update(angle_yaw, millis());
    // _pitch_filter.update(angle_pitch, millis());

    // _frotend.ef_rate_info.x = _yaw_rate_filter.get() + _yaw_filter.slope()*1000.f;
    // // _frotend.ef_rate_info.x = _yaw_filter.slope()*1000.f;
    // _frotend.ef_rate_info.y = _pitch_filter.slope()*1000.f;

    // _frotend.udpate_control_value();

    // // _frotend.display_info_p1 = _yaw_rate_filter.get();
    // // _frotend.display_info_p2 = _yaw_filter.slope()*1000.f;
    // // _frotend.display_info_p3 = _frotend.ef_info.x;
    // // _frotend.display_info_p4 = _frotend.ef_info.y;

    // _frotend.display_info_p1 = _frotend.bf_info.x;
    // _frotend.display_info_p2 = _frotend.bf_info.y;
    // _frotend.display_info_p3 = _frotend.ef_info.x;
    // _frotend.display_info_p4 = _frotend.ef_info.y;
    // _frotend.display_info_new = true;
    // _frotend.display_info_count++;
}


void UTarget_Cam::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
