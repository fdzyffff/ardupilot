#include "Copter.h"

bool UBIM::cmd_add_wp()
{
    if (!copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Fail, no GPS");
        return false;
    }

    if (!uav_unlock) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Fail, need unlock");
        return false;
    }

    FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();

    if (tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx == 1) {
        copter.mode_auto.mission.clear();
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp clear %d", copter.mode_auto.mission.num_commands());
    }

    if (copter.mode_auto.mission.num_commands() == 0) {
        AP_Mission::Mission_Command tmp_cmd;
        Vector3f tmp_pos = Vector3f(
            tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_y - copter.g2.user_parameters.origin_pos_off_x*100.f,
            tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_x - copter.g2.user_parameters.origin_pos_off_y*100.f,
            tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_z + copter.g2.user_parameters.origin_pos_off_z*100.f
            );
        tmp_cmd.content.location = Location(tmp_pos, Location::AltFrame::ABSOLUTE);
        tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
        copter.mode_auto.mission.add_cmd(tmp_cmd);
    }

    if ((uint32_t)copter.mode_auto.mission.num_commands() - 1 == tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx)
    {
        // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Skip, %d == %d", copter.mode_auto.mission.num_commands() - 1, tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx);
        return true;
    }

    if ((uint32_t)(1 + copter.mode_auto.mission.num_commands() - 1) != tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Fail, %d -> %d", copter.mode_auto.mission.num_commands() - 1, tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx);
        return false;
    }

    uav_wp_ok = false;
    AP_Mission::Mission_Command tmp_cmd;
    switch (tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_type) {
        case 0:
        {
            Vector3f tmp_pos = Vector3f(
                (float)tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_y,
                (float)tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_x,
                (float)tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_z
                );
            tmp_cmd.content.location = Location(tmp_pos, Location::AltFrame::ABSOLUTE);
            tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
            tmp_cmd.p1 = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp [%0.1f, %0.1f, %0.1f] %0.1f", tmp_pos.x, tmp_pos.y, tmp_pos.z, (float)tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_speed);
            // tmp_cmd.content.location.lng = 
            // tmp_cmd.content.location.lat = 
            // tmp_cmd.content.location.alt = 
            AP_Param::set_default_by_name("WPNAV_SPEED", (float)tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_speed);
            break;
        }
        case 1:
        {
            Vector3f tmp_pos = Vector3f(
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_y,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_x,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_z
                );
            tmp_cmd.content.location = Location(tmp_pos, Location::AltFrame::ABSOLUTE);
            tmp_cmd.id = MAV_CMD_NAV_TAKEOFF;
            tmp_cmd.p1 = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp [%0.1f, %0.1f, %0.1f]", tmp_pos.x, tmp_pos.y, tmp_pos.z);
            break;
        }
        case 2:
        {
            Vector3f tmp_pos = Vector3f(
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_y,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_x,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_z
                );
            tmp_cmd.content.location = Location(tmp_pos, Location::AltFrame::ABSOLUTE);
            tmp_cmd.id = MAV_CMD_NAV_LAND;
            tmp_cmd.p1 = 1;
            uav_wp_ok = true;
            break;
        }
    }

    if (copter.mode_auto.mission.add_cmd(tmp_cmd)) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Add %d", tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx);
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Fail");
        return false;
    }
    return true;
}

bool UBIM::cmd_set_pos()
{
    return false;
}

bool UBIM::cmd_set_speed()
{
    return false;
}

bool UBIM::cmd_set_alt()
{
    return false;
}

bool UBIM::cmd_set_yaw()
{
    return false;
}

bool UBIM::cmd_set_pos_offset()
{
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff Fail, not in hover");
        return false;
    }
    if (!copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff Fail, bad position");
        return false;
    }
    Vector3f current_pos;
    if (copter.ahrs_view->get_relative_position_NED_origin(current_pos))
    {
        ;
    }
    current_pos.z = -current_pos.z; // from NED to NEU
    current_pos = current_pos * 100.f;
    FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();
    Vector3f tmp_pos = Vector3f(
        (float)tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_y,
        (float)tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_x,
        (float)tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_z
        );
    // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV tmp_pos [%0.1f, %0.1f, %0.1f]", tmp_pos.x, tmp_pos.y, tmp_pos.z);
    // tmp_pos = tmp_pos + current_pos; // NEU
    // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV offset pos [%0.1f, %0.1f, %0.1f]", tmp_pos.x, tmp_pos.y, tmp_pos.z);
    if (copter.mode_guided.set_destination(tmp_pos + current_pos))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff [%0.1f, %0.1f, %0.1f]", tmp_pos.x, tmp_pos.y, tmp_pos.z);
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff Fail");
        return true;
    }
}
