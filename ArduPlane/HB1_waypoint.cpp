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
    if (g2.hb1_num_wp == 0 || g2.hb1_num_interim != 0|| g2.hb1_num_attack != 0) {
        plane.mission.clear();
        plane.mission.add_cmd(tmp_cmd);
        plane.mission.write_home_to_storage();
        g2.hb1_num_wp.set_and_save(0);
        g2.hb1_num_interim.set_and_save(0);
        g2.hb1_num_attack.set_and_save(0);
    }
    if (plane.mission.add_cmd(tmp_cmd)) {
        g2.hb1_num_wp.set_and_save(g2.hb1_num_wp.get()+1);
        gcs().send_text(MAV_SEVERITY_INFO, "WP (%d / %d) saved", g2.hb1_num_wp.get(), plane.mission.num_commands());
    }
}

void Plane::HB1_msg_mission2apm_set_interim_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    HB1_interim_cmd.id = MAV_CMD_NAV_WAYPOINT;
    HB1_interim_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.latitude/tmp_msg.SF_LL);
    HB1_interim_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.longitude/tmp_msg.SF_LL);
    HB1_interim_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

    HB1_interim_cmd.p1 = 0;
    if (g2.hb1_num_attack == 0) {
        if (g2.hb1_num_interim == 0) {
            plane.mission.add_cmd(HB1_interim_cmd);
            g2.hb1_num_interim.set_and_save(1);
        } else {
            plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_interim_cmd);
        }
    } else {
        if (g2.hb1_num_interim == 0) {
            plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_interim_cmd);
            plane.mission.add_cmd(HB1_attack_cmd);
            g2.hb1_num_interim.set_and_save(1);
        } else {
            plane.mission.replace_cmd(plane.mission.num_commands()-2, HB1_interim_cmd);
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Interim (%d / %d) saved", g2.hb1_num_interim.get(), plane.mission.num_commands());
}

void Plane::HB1_msg_mission2apm_set_attack_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    HB1_attack_cmd.id = MAV_CMD_NAV_WAYPOINT;
    HB1_attack_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.latitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.longitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

    HB1_attack_cmd.p1 = 0;
    if (g2.hb1_num_attack == 0) {
        plane.mission.add_cmd(HB1_attack_cmd);
        g2.hb1_num_attack.set_and_save(1);
    } else {
        plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_attack_cmd);
        g2.hb1_num_attack.set_and_save(1);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Attack (%d / %d) saved", g2.hb1_num_attack.get(), plane.mission.num_commands());
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

void Plane::HB1_msg_mission2apm_speed_up_handle() {
    if (HB1_Status.grouped) {
        return;
    }
    float current_spd = aparm.airspeed_cruise_cm.get();
    current_spd += 200.f;
    aparm.airspeed_cruise_cm.set(current_spd);
    gcs().send_text(MAV_SEVERITY_INFO, "Spd : %0.1f", current_spd/100.f);
}

void Plane::HB1_msg_mission2apm_speed_down_handle() {
    if (HB1_Status.grouped) {
        return;
    }
    float current_spd = aparm.airspeed_cruise_cm.get();
    current_spd -= 200.f;
    aparm.airspeed_cruise_cm.set(current_spd);
    gcs().send_text(MAV_SEVERITY_INFO, "Spd : %0.1f", current_spd/100.f);
}

void Plane::HB1_msg_mission2apm_preattack_handle(int32_t time_s) {
    g2.hb1_follow_hover_attack_time.set_and_save(time_s*1000);
    HB1_status_set_HB_Mission_Action(HB1_Mission_PreAttack);
    if (g2.hb1_num_interim > 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "PreAttack received (#%d) [%d ms]", g2.hb1_num_wp+g2.hb1_num_interim, g2.hb1_follow_hover_attack_time.get());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "no interim WP! (#%d)", g2.hb1_num_wp+g2.hb1_num_interim);
    }
}

void Plane::HB1_msg_mission2apm_attack_handle() {
    if ( (HB1_Status.state == HB1_Mission_PreAttack) || (HB1_Status.state == HB1_Mission_Hover2) ) {
        if (g2.hb1_num_attack > 0) {
            HB1_status_set_HB_Mission_Action(HB1_Mission_Attack);
            gcs().send_text(MAV_SEVERITY_INFO, "Attack received (#%d)", g2.hb1_num_wp+g2.hb1_num_interim+1);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "no attack WP!");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Not in PreAttack!");
    }
}

void Plane::HB1_msg_mission2apm_EngineSTART_handle() {
    if (!arming.is_armed()) {
        // check if need to re send start cmd
        if ((fabsf(plane.HB1_Power.HB1_engine_rpm) < 50.f) || (g2.hb1_rpm_used == 0)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine ground start");
            HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineSTART, true);
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground start");
    }
}

void Plane::HB1_msg_mission2apm_EngineOFF_handle() {
    gcs().send_text(MAV_SEVERITY_INFO, "Engine ground stop");
    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineOFF, true);
}

void Plane::HB1_msg_mission2apm_EngineFULL_handle() {
    if (!arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine ground full");
        HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineFULL, true);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground full");
    }
}

void Plane::HB1_msg_mission2apm_EngineMID_handle() {
    if (!arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine ground mid");
        HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineMID, true);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground mid");
    }
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
