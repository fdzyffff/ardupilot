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

void UMission::handle_msg(const mavlink_message_t &msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
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
                        gcs().send_text(MAV_SEVERITY_INFO, "Do group %0.0f, %0.0f, %0.0f", packet.param1, packet.param2, packet.param3);
                        group_state.in_group = ((int16_t)packet.param2 == 1);
                        group_state.type = (int16_t)packet.param3;
                        plane.g2.group_distance.set(packet.param4);
                        plane.g2.user_group_id.set(group_state.type);
                        do_group_init();
                        gcs().send_text(MAV_SEVERITY_INFO, "G[%d, %d, %d]", (int16_t)packet.param2, (int16_t)packet.param3, (int16_t)packet.param4);
                        break;
                    case 3:
                        if (_role == Mission_Role::Leader) {
                            do_offboard_control(packet.param2);
                        }
                        break;
#if HAL_QUADPLANE_ENABLED
                    case 4:
                        plane.set_mode(plane.mode_qtakeoff, ModeReason::GCS_COMMAND);
                        break;
#endif
                }
                break;
            default:
                break;
        }
        switch (packet.command) {
            case MAV_CMD_USER_2:
                switch ((int16_t)packet.param1) {
                    default:
                        break;
                    case 1:
                        if (plane.uattack._cam_port_type == 1) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Test [%f, %f]", packet.param2, packet.param3);
                            plane.uattack._UCam_ptr->handle_info(packet.param2, packet.param3);
                        }
                        break;
                }
                break;
            default:
                break;
        }
    }
}

void UMission::do_offboard_control(float p2) {

    int16_t option = p2;
    switch (option) {
        default:
            break;
        case 1:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.change_target_altitude_cm(1000);
            break;
        case 2:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.change_target_altitude_cm(-1000);
            break;
        case 3:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.change_target_heading_cd(-3000);
            break;
        case 4:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.change_target_heading_cd(3000);
            break;
        case 5:
            if (plane.control_mode != &plane.mode_loiter) {
                plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
            }
            plane.mode_loiter.do_circle_ccw();
            break;
        case 6:
            if (plane.control_mode != &plane.mode_loiter) {
                plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
            }
            plane.mode_loiter.do_circle_cw();
            break;
        case 7:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.do_cmd_1();
            break;
        case 8:
            if (plane.control_mode != &plane.mode_cruise) {
                plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
            }
            plane.mode_cruise.do_cmd_2();
            break;
    }

}

void UMission::do_group_init() {
    if (group_state.in_group) {
        if (_role == Mission_Role::Leader) {
            set_mode(plane.mode_auto);
            plane.gcs().send_text(MAV_SEVERITY_INFO, "In group, Leader");
        }
        if (_role == Mission_Role::Follower) {
            if (!plane.ufollow.is_active()) {
                set_mode(plane.mode_auto);
            } else {
                set_mode(plane.mode_guided);
            }
            plane.gcs().send_text(MAV_SEVERITY_INFO, "In group, Follower");
        }
    } else {
        set_mode(plane.mode_auto);
        plane.gcs().send_text(MAV_SEVERITY_INFO, "Exit group");
    }
}


void UMission::keep_group() {
    if (group_state.in_group) {
        if (_role == Mission_Role::Leader) {
            set_mode(plane.mode_auto);
        }
        if (_role == Mission_Role::Follower) {
            if (!plane.ufollow.is_active()) {
                set_mode(plane.mode_auto);
            } else {
                set_mode(plane.mode_guided);
            }
        }
    }
}

void UMission::do_set_leader(float p2) {
    _leader_id = (int16_t)p2;
    update_leader();
    keep_group();
}

void UMission::update_leader() {
    Mission_Role new_role;
    if (_leader_id == plane.g.sysid_this_mav) {
        new_role = Mission_Role::Leader;
    } else {
        new_role = Mission_Role::Follower;
    }
    if (new_role != _role) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set Role: %d", (uint8_t)new_role);
    }
    _role = new_role;
}

void UMission::update_group() {
    if (group_state.in_group) {
        if (group_state.mode != plane.control_mode) {
            group_state.in_group = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Mode changed remotely, group exit.");
            plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
            return;
        }
    }

    if (group_state.in_group) {
        keep_group();
    }
}

void UMission::set_mode(Mode& new_mode) {
    // gcs().send_text(MAV_SEVERITY_INFO, "mode num %d %d",(&new_mode)->mode_number(), plane.control_mode->mode_number());
    if (&new_mode == plane.control_mode) {return;}
    if (plane.set_mode(new_mode, ModeReason::GCS_COMMAND)) {
        group_state.mode = &new_mode;
    } else {
        group_state.in_group = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Mode change failed, group exit.");
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "mode num %d %d",(&new_mode)->mode_number(), plane.control_mode->mode_number());
}