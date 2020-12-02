#include "Plane.h"

void Plane::HB1_msg_mission2apm_takeoff_handle() {
    if (HB1_Status.already_takeoff) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already Takeoff");}
    else {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Takeoff);
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff received");
    }
}
void Plane::HB1_msg_mission2apm_set_wp_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    AP_Mission::Mission_Command tmp_cmd;
    tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

    tmp_cmd.p1 = 0;
    if (HB1_Status.num_wp == 0) {
        plane.mission.clear();
        plane.mission.add_cmd(tmp_cmd);
        plane.mission.write_home_to_storage();
    }
    if (plane.mission.add_cmd(tmp_cmd)) {
        HB1_Status.num_wp += 1;
        gcs().send_text(MAV_SEVERITY_INFO, "WP (%d / %d) saved", HB1_Status.num_wp, plane.mission.num_commands());
    }
}

void Plane::HB1_msg_mission2apm_set_interim_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    AP_Mission::Mission_Command tmp_cmd;

    tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.latitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.longitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

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

void Plane::HB1_msg_mission2apm_set_attack_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    HB1_attack_cmd.id = MAV_CMD_NAV_WAYPOINT;
    HB1_attack_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.latitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.longitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

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

void Plane::HB1_msg_mission2apm_away_handle(HB1_mission2apm &tmp_msg) {
    HB1_Status.grouped = false;
    uint16_t target_wp_index = tmp_msg._msg_1.content.msg.leader_target_id;

    if (target_wp_index < mission.num_commands()) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_WP);
        gcs().send_text(MAV_SEVERITY_INFO, "Away received (#%d)", target_wp_index);
        auto_state.next_wp_crosstrack = false;
        mission.set_current_cmd(target_wp_index);
    } else {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
        gcs().send_text(MAV_SEVERITY_INFO, "Away, hover");
    }
}

void Plane::HB1_msg_mission2apm_attack_handle() {
    if (HB1_Status.num_attack > 0 || HB1_Status.num_interim > 0) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Attack);
        gcs().send_text(MAV_SEVERITY_INFO, "Attack received (#%d)", HB1_Status.num_wp+1);
    } else {
        HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
        gcs().send_text(MAV_SEVERITY_INFO, "no attack WP!");
    }
}

void Plane::HB1_msg_mission2apm_EngineON_handle() {
    if (!arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine ground start");
        HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineSTART);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground start");
    }
}

void Plane::HB1_msg_mission2apm_EngineOFF_handle() {
    gcs().send_text(MAV_SEVERITY_INFO, "Engine ground stop");
    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineOFF);
}

void Plane::HB1_msg_mission2apm_EngineTest_handle() {
    gcs().send_text(MAV_SEVERITY_INFO, "Engine ground test");
    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineTEST);
}

void Plane::HB1_msg_mission2apm_Disarm_handle() {
    gcs().send_text(MAV_SEVERITY_INFO, "plane disarming");
    arming.disarm();
}

void Plane::HB1_msg_mission2apm_ServoTest_handle() {
    if (arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first!");
        return;
    }
    if (set_mode(mode_servotest, MODE_REASON_UNAVAILABLE)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Servo ground test");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Servo ground test failed");
    }
}
