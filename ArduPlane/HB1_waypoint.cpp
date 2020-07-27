#include "Plane.h"

void Plane::HB1_msg_mission2apm_takeoff_handle() {
    HB1_status_set_HB_Mission_Action(HB1_Mission_Takeoff);
    gcs().send_text(MAV_SEVERITY_INFO, "Takeoff received");
}
void Plane::HB1_msg_mission2apm_wp_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    AP_Mission::Mission_Command& tmp_cmd;
    if (HB1_Status.num_wp == 0) {
        plane.mission.clear();
        plane.mission.write_home_to_storage();
    }
    tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.alt = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt/tmp_msg.SF_ALT);

    tmp_cmd.content.location.options = 0;
    //if want to set alt from home point add following line
    tmp_cmd.content.location.flags.relative_alt = 1; //alt from home point
    tmp_cmd.p1 = 0;
    if (plane.mission.add_cmd(tmp_cmd)) {
        HB1_Status.num_wp += 1;
        gcs().send_text(MAV_SEVERITY_INFO, "WP (%d / %d) saved", HB1_Status.num_wp, plane.mission.num_commands());
    }
}

void Plane::HB1_msg_mission2apm_interim_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    AP_Mission::Mission_Command& tmp_cmd;

    tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.alt = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt/tmp_msg.SF_ALT);

    tmp_cmd.content.location.options = 0;
    //if want to set alt from home point add following line
    tmp_cmd.content.location.flags.relative_alt = 1; //alt from home point
    tmp_cmd.p1 = 0;
    if (HB1_Status.num_attack == 0) {
        plane.mission.add_cmd(tmp_cmd);
        HB1_Status.num_interim += 1;
    } else {
        plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_cmd);
        plane.mission.add_cmd(HB1_attack_cmd);
        HB1_Status.num_interim += 1;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Interim (%d / %d) saved", HB1_Status.num_interim, plane.mission.num_commands());
}

void Plane::HB1_msg_mission2apm_attack_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    HB1_attack_cmd.id = MAV_CMD_NAV_WAYPOINT;
    HB1_attack_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.alt = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt/tmp_msg.SF_ALT);

    HB1_attack_cmd.content.location.options = 0;
    //if want to set alt from home point add following line
    HB1_attack_cmd.content.location.flags.relative_alt = 1; //alt from home point
    HB1_attack_cmd.p1 = 0;
    if (HB1_Status.num_attack == 0) {
        plane.mission.add_cmd(HB1_attack_cmd);
        HB1_Status.num_attack = 1;
    } else {
        plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_attack_cmd);
        HB1_Status.num_attack = 1;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Attack (%d / %d) saved", HB1_Status.num_attack, plane.mission.num_commands());
}

void Plane::HB1_msg_mission2apm_away_handle() {
    if (HB1_Status.Attack > 0) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Attack);
    } else {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Away received");
}
