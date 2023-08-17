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
    update_group();
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
                        group_state.in_group = ((int16_t)packet.param2 == 1);
                        group_state.type = (int16_t)packet.param3;
                        do_group();
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

void UMission::do_group() {
    if (group_state.in_group) {
        if (_role == Mission_Role::Leader) {
            if (group_state.type == 1) {
                set_mode(mode_cruise);
            } else {
                set_mode(mode_auto);
            }
        } else {
            set_mode(mode_guided);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "group engaged.");
    } else {
        plane.set_mode(mode_auto, ModeReason::GCS_COMMAND);
        gcs().send_text(MAV_SEVERITY_INFO, "group exit.");
    }
}

void UMission::do_set_leader(float p2) {
    _leader_id = (int16_t)p2;
}

void UMission::update_leader() {
    if (_leader_id == plane.g.sysid_this_mav) {
        _role = Mission_Role::Leader;
    } else {
        _role = Mission_Role::Follower;
    }
    if (group_state.in_group) {
        do_group();
    }
}

void UMission::update_group() {
    if (group_state.in_group) {
        if (group_state.mode != plane.control_mode) {
            group_state.in_group = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Mode changed remotely, group exit.");
        }
    } else {
        return;
    }
    // failsafe check in case no leader position message.
    if (_role == Mission_Role::Follower) {
        if (!plane.ufollow.is_active()) {
            set_mode(mode_auto);
        } else {
            set_mode(mode_guided);
        }
    }
}

void UMission::set_mode(Mode& new_mode) {
    if (plane.set_mode(new_mode, ModeReason::GCS_COMMAND)) {
        group_state.mode = new_mode;
    } else {
        group_state.in_group = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Mode change failed, group exit.");
    }
}