#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UTarget_Mav::UTarget_Mav(UAttack &frotend_in):
    UTarget_Base(frotend_in)
{
    _yaw_sample_filter.set_cutoff_frequency(30.f, 2.f);
    _pitch_sample_filter.set_cutoff_frequency(30.f, 2.f);
    return;
}

bool UTarget_Mav::init() {
    return true;
}

void UTarget_Mav::update() {
    static uint32_t last_update_ms = millis();

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

void UTarget_Mav::do_cmd() {
    ;
}

bool UTarget_Mav::is_valid() {
    return _valid;
}

void UTarget_Mav::handle_info(float p1, float p2) {
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

void UTarget_Mav::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                if (is_equal(packet.param7, 1.0f)) {
                    float p1 = packet.param5;
                    float p2 = packet.param6;
                    handle_info(p1, p2);
                }
                break;
            default:
                break;
        }
    }
}

void UTarget_Mav::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
