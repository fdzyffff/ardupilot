#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UTarget_Loc::UTarget_Loc(UAttack &frotend_in):
    UTarget_Base(frotend_in)
{
    _have_target = false;
    _yaw_sample_filter.set_cutoff_frequency(30.f, 2.f);
    _pitch_sample_filter.set_cutoff_frequency(30.f, 2.f);
    return;
}

bool UTarget_Loc::init() {
    return true;
}

void UTarget_Loc::update() {
    static uint32_t last_update_ms = millis();

    if (_have_target && (millis() - _last_ms) > 33) {
        Vector3f off_ef = plane.current_loc.get_distance_NED(plane.mode_attack_loc.target_loc);

        _frotend.display_info.p1 = off_ef.z;
        _frotend.display_info.p2 = plane.mode_attack_loc.target_loc.alt/100;
        Matrix3f tmp_earth_m;
        tmp_earth_m.from_euler(plane.ahrs.get_roll(), plane.ahrs.get_pitch(), plane.ahrs.get_yaw());
        tmp_earth_m.transpose();
        Vector3f off_bf = tmp_earth_m*off_ef;
        off_bf.normalized();

        float p1 = degrees(wrap_180(atan2f( off_bf.y, off_bf.x))); // x-axis, degrees
        float p2 = degrees(wrap_180(atan2f(-off_bf.z, off_bf.xy().length()))); // y-axis, degrees

        handle_info(p1, p2);
    }

    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((plane.g2.attack_timeout > 0) && (tnow - _last_ms > (uint32_t)plane.g2.attack_timeout)) {
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

void UTarget_Loc::do_cmd() {
    ;
}

bool UTarget_Loc::is_valid() {
    return _valid;
}

void UTarget_Loc::handle_info(float p1, float p2) {
    _valid = true;
    _last_ms = millis();

    _frotend.display_info.p3 = p1;
    _frotend.display_info.p4 = p2;

    float _roll = plane.ahrs.get_roll();
    float _pitch = plane.ahrs.get_pitch();
    float _yaw = plane.ahrs.get_yaw();
    // if (!copter.udelay.get_idx(5-1, _roll, _pitch, _yaw)) {
    //     _roll = plane.ahrs.get_roll();
    //     _pitch = plane.ahrs.get_pitch();
    //     _yaw = plane.ahrs.get_yaw();
    // }

    _frotend.bf_info.x = p1; // yaw degree
    _frotend.bf_info.y = p2; // pitch degree

    float bf_dist = 100.0f;
    float bf_z    = -bf_dist*sinf(radians(p2));
    float bf_xy   =  bf_dist*cosf(radians(p2));
    float bf_y    =  bf_xy*sinf(radians(p1));
    float bf_x    =  bf_xy*cosf(radians(p1));
    Vector3f bf_unit = Vector3f(bf_x, bf_y, bf_z);
    bf_unit.normalized();

    Matrix3f tmp_body_m;
    tmp_body_m.from_euler(_roll, _pitch, _yaw);
    Vector3f ef_unit = tmp_body_m*bf_unit;

    float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
    float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));

    _frotend.ef_info.x = angle_yaw;
    _frotend.ef_info.y = angle_pitch;

    float delta_yaw = wrap_180(wrap_360(angle_yaw) - wrap_360(_last_yaw));
    _last_yaw = angle_yaw;
    _last_yaw_sample += delta_yaw;

    _yaw_sample_filter.apply(_last_yaw_sample);
    _pitch_sample_filter.apply(angle_pitch);

    _yaw_filter.update(_yaw_sample_filter.get(), millis());
    _pitch_filter.update(_pitch_sample_filter.get(), millis());

    _frotend.ef_rate_info.x = _yaw_filter.slope()*1000.f;
    _frotend.ef_rate_info.y = _pitch_filter.slope()*1000.f;

    _frotend.display_info.new_data = true;
    _frotend.display_info.count++;

    _frotend.udpate_control_value();
}

void UTarget_Loc::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        // decode packet
        gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                plane.mode_attack_loc.target_loc.lat = packet.x;
                plane.mode_attack_loc.target_loc.lng = packet.y;
                // plane.mode_attack_loc.target_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABOVE_HOME);
                // plane.mode_attack_loc.target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
                plane.mode_attack_loc.target_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABSOLUTE);
                _have_target = true;
                // gcs().send_text(MAV_SEVERITY_INFO,"x %f", (float)packet.x);
                // gcs().send_text(MAV_SEVERITY_INFO,"y %f", (float)packet.y);
                // gcs().send_text(MAV_SEVERITY_INFO,"z %f", (float)packet.z);
                break;
            default:
                break;
        }
    }

}

void UTarget_Loc::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
