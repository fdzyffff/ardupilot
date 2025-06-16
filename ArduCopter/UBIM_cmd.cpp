#include "Copter.h"

bool UBIM::cmd_add_wp()
{
    FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();
    if ((uint32_t)(1 + copter.mode_auto.mission.num_commands()) != tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Fail, %d -> %d", copter.mode_auto.mission.num_commands(), tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx);
        return false;
    }

    if ((uint32_t)copter.mode_auto.mission.num_commands() == tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Skip, %d == %d", copter.mode_auto.mission.num_commands(), tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_idx);
        return true;
    }

    AP_Mission::Mission_Command tmp_cmd;
    switch (tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_type) {
        case 0:
        {
            Vector3f tmp_pos = Vector3f(
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_y,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_x,
                tmp_msg._msg_1.content.msg.plat_input_param.input_56H.wp_pos_z
                );
            tmp_cmd.content.location = Location(tmp_pos, Location::AltFrame::ABSOLUTE);
            tmp_cmd.id = MAV_CMD_NAV_TAKEOFF;
            // tmp_cmd.content.location.lng = 
            // tmp_cmd.content.location.lat = 
            // tmp_cmd.content.location.alt = 
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
            tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
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
            break;
        }
    }

    if (copter.mode_auto.mission.add_cmd(tmp_cmd)) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp Add");
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
    FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();
    Vector3f tmp_pos = Vector3f(
        tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_y,
        tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_x,
        tmp_msg._msg_1.content.msg.plat_input_param.input_7AH.offset_pos_z
        );
    tmp_pos = tmp_pos + current_pos; // NEU
    if (copter.mode_guided.set_destination(tmp_pos))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff");
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV set posoff Fail");
        return true;
    }
}
