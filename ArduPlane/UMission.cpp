#include "Plane.h"

UMission::UMission() {
    ;
}

void UMission::init() {
    _role = Mission_Role::Leader;
    _leader_id = plane.g.sysid_this_mav;
}

void UMission::update() {
    update_leader();
    // failsafe in case connection lost
    if (_role == Mission_Role::Follower) {
        if (!plane.ufollow.is_active()) {
            plane.set_mode(mode_auto, ModeReason::GCS_COMMAND);
        }
    }
}

void UMission::handle_info(const mavlink_command_long_t* packet) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch (packet.command) {
            case MAV_CMD_USER_1:
                switch ((int16_t)packet.param1) {
                    default:
                        break;
                    case 1:
                        do_set_leader(packet.param2);
                        break;
                    case 2:
                        do_group(packet.param2);
                        break;
                    case 3:
                        do_offboard_cruise(packet.param2);
                        break;
                }
                break;
            default:
                break;
        }
    }
}

void UMission::do_offboard_cruise(float p2) {
    if (control_mode != &mode_cruise) {
        plane.set_mode(mode_cruise, ModeReason::GCS_COMMAND);
    }

    int16_t option = p2;
    switch (option) {
        default:
            break;
        case 1:
            plane.mode_cruise.change_target_altitude_cm(1000);
            break;
        case 2:
            plane.mode_cruise.change_target_altitude_cm(-1000);
            break;
        case 3:
            plane.mode_cruise.change_target_heading_cd(-3000);
            break;
        case 4:
            plane.mode_cruise.change_target_heading_cd(3000);
            break;
    }

}

void UMission::do_group(bool release) {
    if (!release) {
        if (_role == Mission_Role::Leader) {
            plane.set_mode(mode_auto, ModeReason::GCS_COMMAND);
        } else {
            plane.set_mode(mode_guided, ModeReason::GCS_COMMAND);
        }
    } else {
        _role = Mission_Role::Leader;
        _leader_id = plane.g.sysid_this_mav;
        plane.set_mode(mode_auto, ModeReason::GCS_COMMAND);
    }
}

void UMission::do_set_leader() {
    _leader_id = (int16_t)packet.param1;
    update_leader();
}

void UMission::update_leader() {
    if (_leader_id == plane.g.sysid_this_mav) {
        _role = Mission_Role::Leader;
    } else {
        _role = Mission_Role::Follower;
    }
}

