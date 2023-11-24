#include "Plane.h"

UFollow::UFollow()
{
    _active = false;
}

void UFollow::init()
{
    if (!plane.g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
        gcs().send_text(MAV_SEVERITY_INFO, "Set FOLL_ENABLE and reboot");
    }
}

void UFollow::handle_my_follow_msg(const mavlink_message_t &msg)
{
    // if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    //     // decode packet
    //     mavlink_command_int_t packet;
    //     mavlink_msg_command_int_decode(&msg, &packet);
    //     switch (packet.command) {
    //         case MAV_CMD_USER_1:
    //             if (plane.umission.get_leader_id() < 9 && plane.umission.get_leader_id() == msg.sysid) {
    //                 _target_vel = packet.param1; // m/s
    //                 _target_bearing = packet.param2; // degree
    //                 _raw_target_loc.lat = packet.x; // 1e7 degree
    //                 _raw_target_loc.lng = packet.y; // 1e7 degree
    //                 _raw_target_loc.alt = (int32_t)packet.z; // cm
    //                 _last_update_ms = millis();
    //             }
    //             break;
    //         default:
    //             break;
    //     }
    // }
    plane.g2.follow.handle_msg(msg);
}

void UFollow::update()
{
    bool valid_target = false;
    if (plane.g2.follow.have_target()) {
        if (plane.g2.follow.get_target_location_and_velocity(_raw_target_loc, _target_vel)) {
            if (plane.g2.follow.get_target_heading_deg(_target_bearing)) {
                valid_target = true;
                _raw_target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
            }
        }
    }
    if (!valid_target) {
        if (_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Follow MSG Lost!");
            _active = false;
        }
    } else {
        if (!_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Follow MSG Received!");
            _active = true;
        }
    }
}

void UFollow::get_target_pos(Location &loc)
{
    Vector3f offset_position = ugroup.get_offset(plane.g.sysid_this_mav, plane.umission.get_leader_id(), plane.g2.group_distance);
    // gcs().send_text(MAV_SEVERITY_INFO, "dist: %f", offset_position.length());
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(_target_bearing));
    offset_position = tmp_m*offset_position;

    loc = _raw_target_loc;
    loc.offset(offset_position.x, offset_position.y);
    loc.alt += (int32_t)(offset_position.z*100.f);
    // float delta_t = 0.01f*(float)(millis() - _last_update_ms);
    // float dist_offset = _target_vel * delta_t;
    // loc.offset_bearing(_target_bearing, dist_offset);
}

void UFollow::print()
{
    gcs().send_text(MAV_SEVERITY_INFO, "[%d](%0.2f, %0.2f)", _active, _target_vel.xy().length(), _target_bearing);
}
