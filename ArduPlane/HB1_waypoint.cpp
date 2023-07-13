#include "Plane.h"

void Plane::HB1_wp_init() {
    HB1_Status.wp_to_renew = false;
    bool read_wp = false;
    bool read_interim = false;
    bool read_attack = false;
    if (g2.hb1_num_wp != 0) {
        read_wp = plane.mission.read_cmd_from_storage(g2.hb1_num_wp,HB1_lastWP_cmd); 
    }
    if (g2.hb1_num_interim != 0) {
        read_interim = plane.mission.read_cmd_from_storage(g2.hb1_num_wp+g2.hb1_num_interim,HB1_interim_cmd);     
    }
    if (g2.hb1_num_land != 0) {
        read_attack = plane.mission.read_cmd_from_storage(g2.hb1_num_wp+g2.hb1_num_interim+g2.hb1_num_land,HB1_attack_cmd);     
    }
    if (!read_wp || !read_interim || !read_attack) {
        g2.hb1_num_wp.set_and_save(0);
        g2.hb1_num_interim.set_and_save(0);
        g2.hb1_num_land.set_and_save(0);
        plane.mission.clear();
        gcs().send_text(MAV_SEVERITY_INFO, "clean mis %d(%d):%d(%d):%d(%d)",read_wp,g2.hb1_num_wp,read_interim,g2.hb1_num_interim,read_attack,g2.hb1_num_land);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "recover mis %d(%d):%d(%d):%d(%d)",read_wp,g2.hb1_num_wp,read_interim,g2.hb1_num_interim,read_attack,g2.hb1_num_land);
        HB1_Status.wp_to_renew = true;
    }
}

void Plane::HB1_msg_mission2apm_takeoff_handle() {
    if (HB1_Status.already_takeoff) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already Takeoff");
        return;
    }
    if (HB1_Power.state == HB1_PowerAction_None) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is not running");
        return;
    }
    if (HB1_Power.state == HB1_PowerAction_GROUND_EngineSTART) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is booting");
        return;
    }
    HB1_status_set_HB_Mission_Action(HB1_Mission_Takeoff);
    gcs().send_text(MAV_SEVERITY_INFO, "Takeoff received");
}

void Plane::HB1_msg_mission2apm_set_wp_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    AP_Mission::Mission_Command tmp_cmd;
    tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude/tmp_msg.SF_LL);
    tmp_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

    uint16_t line_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index;
    uint16_t point_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index;  

    line_index = MIN(0xFF,line_index);// now for control_id
    point_index = MIN(0xFF,point_index);
    tmp_cmd.p1 = (line_index << 8) | (point_index & 0x00FF);

    if (HB1_Status.wp_to_renew || g2.hb1_num_wp == 0 || g2.hb1_num_interim != 0|| g2.hb1_num_land != 0) {
        plane.mission.clear();
        plane.mission.add_cmd(tmp_cmd);
        plane.mission.write_home_to_storage();
        g2.hb1_num_wp.set_and_save(0);
        g2.hb1_num_interim.set_and_save(0);
        g2.hb1_num_land.set_and_save(0);
        HB1_Status.wp_to_renew = false;
    }
    if (plane.mission.add_cmd(tmp_cmd)) {
        g2.hb1_num_wp.set_and_save(g2.hb1_num_wp.get()+1);
        gcs().send_text(MAV_SEVERITY_INFO, "[%d] WP (%d / %d) saved", line_index, g2.hb1_num_wp.get(), plane.mission.num_commands());
        HB1_lastWP_cmd = tmp_cmd;
    }
}

void Plane::HB1_msg_mission2apm_set_interim_handle() {
    if (HB1_Status.state == HB1_Mission_Hover2 || HB1_Status.state == HB1_Mission_Land) {return;}
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    uint16_t line_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index;
    uint16_t point_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index;  

    line_index = MIN(0xFF,line_index);// now for control_id
    point_index = MIN(0xFF,point_index);

    if (point_index < 1) { return; }
    if (point_index > 2) { return; }

    AP_Mission::Mission_Command tmp_interim_cmd;
    tmp_interim_cmd.id = MAV_CMD_NAV_WAYPOINT;
    tmp_interim_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.latitude/tmp_msg.SF_LL);
    tmp_interim_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.longitude/tmp_msg.SF_LL);
    tmp_interim_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);
    tmp_interim_cmd.p1 = (line_index << 8) | (point_index & 0x00FF);

    if (point_index == 1) {
        if (g2.hb1_num_land == 0) {
            if (g2.hb1_num_interim == 0) {
                plane.mission.add_cmd(tmp_interim_cmd);
                g2.hb1_num_interim.set_and_save(1);
            } else if (g2.hb1_num_interim == 1) {
                plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_interim_cmd);
            } else if (g2.hb1_num_interim == 2) {
                plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_interim_cmd);
            }
        } else {
            if (g2.hb1_num_interim == 0) {
                plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_interim_cmd);
                plane.mission.add_cmd(HB1_attack_cmd);
                g2.hb1_num_interim.set_and_save(1);
            } else if (g2.hb1_num_interim == 1) {
                plane.mission.replace_cmd(plane.mission.num_commands()-2, tmp_interim_cmd);
            } else if (g2.hb1_num_interim == 2) {
                plane.mission.replace_cmd(plane.mission.num_commands()-2, tmp_interim_cmd);
            }
        }
        // udpate HB1_interim_cmd
        HB1_interim_cmd = tmp_interim_cmd;
    } else if (point_index == 2) {
        if (g2.hb1_num_land == 0) {
            if (g2.hb1_num_interim == 0) {
                plane.mission.add_cmd(tmp_interim_cmd);
                g2.hb1_num_interim.set_and_save(1);
                // udpate HB1_interim_cmd
                HB1_interim_cmd = tmp_interim_cmd;
            } else if (g2.hb1_num_interim == 1) {
                plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_interim_cmd);
                plane.mission.add_cmd(HB1_interim_cmd);
                g2.hb1_num_interim.set_and_save(2);
            } else if (g2.hb1_num_interim == 2) {
                plane.mission.replace_cmd(plane.mission.num_commands()-2, tmp_interim_cmd);
            }
        } else {
            if (g2.hb1_num_interim == 0) {
                plane.mission.replace_cmd(plane.mission.num_commands()-1, tmp_interim_cmd);
                plane.mission.add_cmd(HB1_attack_cmd);
                g2.hb1_num_interim.set_and_save(1);
                HB1_interim_cmd = tmp_interim_cmd;
            } else if (g2.hb1_num_interim == 1) {
                plane.mission.replace_cmd(plane.mission.num_commands()-2, tmp_interim_cmd);
                plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_interim_cmd);
                plane.mission.add_cmd(HB1_attack_cmd);
                g2.hb1_num_interim.set_and_save(2);
            } else if (g2.hb1_num_interim == 2) {
                plane.mission.replace_cmd(plane.mission.num_commands()-3, tmp_interim_cmd);
            }
        }
    }

    if ((HB1_Status.state == HB1_Mission_PreLand) && (control_mode == &mode_auto) ) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_PreLand,true);
        gcs().send_text(MAV_SEVERITY_INFO, "Interim (%d / %d) reset and saved", g2.hb1_num_interim.get(), plane.mission.num_commands());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Interim (%d / %d) saved", g2.hb1_num_interim.get(), plane.mission.num_commands());
    }
}

void Plane::HB1_msg_mission2apm_set_land_handle() {
    if (HB1_Status.state == HB1_Mission_Hover2 || HB1_Status.state == HB1_Mission_Land) {return;}
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();

    HB1_attack_cmd.id = MAV_CMD_NAV_LAND;
    HB1_attack_cmd.content.location.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.latitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.longitude/tmp_msg.SF_LL);
    HB1_attack_cmd.content.location.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);

    uint16_t line_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index;
    uint16_t point_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index;  

    line_index = MIN(0xFF,line_index);// empty
    point_index = MIN(0xFF,point_index);
    HB1_attack_cmd.p1 = (line_index << 8) | (point_index & 0x00FF);

    if (g2.hb1_num_land == 0) {
        plane.mission.add_cmd(HB1_attack_cmd);
        g2.hb1_num_land.set_and_save(1);
    } else {
        plane.mission.replace_cmd(plane.mission.num_commands()-1, HB1_attack_cmd);
        g2.hb1_num_land.set_and_save(1);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Land (%d / %d) saved", g2.hb1_num_land.get(), plane.mission.num_commands());
}

void Plane::HB1_msg_mission2apm_Search_wp_handle() {
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    HB1_Status.search_wp = true;
    HB1_Status.search_line_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_searchwp.line_index;
    HB1_Status.search_id = 1;
    HB1_Status.search_ms = millis();
    HB1_Status.remote_index = tmp_msg._msg_1.content.msg.remote_index;
}

void Plane::HB1_msg_mission2apm_Search_wp_pack() {
    if (!HB1_Status.search_wp) {return;}
    uint32_t tnow = millis();
    if (tnow - HB1_Status.search_ms >= 500) {
        HB1_msg_mission2apm_Search_wp_pack_next();
        return;
    }
    AP_Mission::Mission_Command tmp_cmd;
    if (!mission.read_cmd_from_storage(HB1_Status.search_id , tmp_cmd)) {
        HB1_msg_mission2apm_Search_wp_pack_next();
        return;
    }
    // if (HIGHBYTE(tmp_cmd.p1) == HB1_Status.search_line_index) {
    if (HB1_Status.search_line_index == 1) {
        HB1_apm2mission &new_msg = HB1_uart_mission.get_msg_apm2mission();
        // new_msg._msg_1.content.msg.remote_index = HB1_Status.remote_index;
        // new_msg._msg_1.content.msg.line_index = 1;// HIGHBYTE(tmp_cmd.p1);
        // new_msg._msg_1.content.msg.point_index = HB1_Status.search_id;//LOWBYTE(tmp_cmd.p1);
        new_msg._msg_1.content.msg.longitude = (int32_t)((double)tmp_cmd.content.location.lng * new_msg.SF_LL);
        new_msg._msg_1.content.msg.latitude = (int32_t)((double)tmp_cmd.content.location.lat * new_msg.SF_LL);
        int32_t alt_target = 0;
        // if (!tmp_cmd.content.location.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_target)) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "get waypoint %d alt failed, set to 0", new_msg._msg_1.content.msg.point_index);
        // }
        new_msg._msg_1.content.msg.alt = (int16_t)((float)alt_target * new_msg.SF_ALT);
        // new_msg._msg_1.content.msg.control_id = HIGHBYTE(tmp_cmd.p1);
    }
    return;
}

void Plane::HB1_msg_mission2apm_Search_wp_pack_next() {
    uint32_t tnow = millis();
    HB1_Status.search_ms = tnow;
    gcs().send_text(MAV_SEVERITY_INFO, "sent waypoint %d ", HB1_Status.search_id);
    HB1_Status.search_id++;
    if (HB1_Status.search_id > g2.hb1_num_wp.get() + g2.hb1_num_interim.get() + g2.hb1_num_land.get()) {
        HB1_Status.search_wp = false;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "can not find waypoint %d ", target_wp_id);
}


void Plane::HB1_msg_mission2apm_away_handle(HB1_mission2apm &tmp_msg) {
    HB1_Status.grouped = false;
    uint16_t target_wp_index = tmp_msg._msg_1.content.msg.leader_target_id;

    if (target_wp_index < mission.num_commands()) {
        if (target_wp_index > g2.hb1_num_wp) {target_wp_index = g2.hb1_num_wp;}
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

void Plane::HB1_msg_mission2apm_preland_handle(int32_t time_s) {
    if (g2.hb1_pilot_type == 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "Not Land type (%d)", g2.hb1_pilot_type.get());
        return;
    }
    g2.hb1_follow_hover_attack_time.set_and_save(time_s*1000);
    HB1_status_set_HB_Mission_Action(HB1_Mission_PreLand);
    if (g2.hb1_num_interim > 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "PreLandreceived (#%d) [%d ms]", g2.hb1_num_wp.get()+1, g2.hb1_follow_hover_attack_time.get());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "no interim WP! (#%d)", g2.hb1_num_wp.get()+g2.hb1_num_interim.get());
    }
}

void Plane::HB1_msg_mission2apm_land_handle() {
    if ( (HB1_Status.state == HB1_Mission_PreLand) || (HB1_Status.state == HB1_Mission_Hover2) ) {
        if (g2.hb1_num_land > 0) {
            HB1_status_set_HB_Mission_Action(HB1_Mission_Land);
            gcs().send_text(MAV_SEVERITY_INFO, "Land received (#%d)", g2.hb1_num_wp+g2.hb1_num_interim+1);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "no Land WP!");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Not in PreLand!");
    }
}

void Plane::HB1_msg_mission2apm_RocketON_handle() {
    if (!arming.is_armed()) {
        plane.HB1_status_set_HB_Power_Action(plane.HB1_PowerAction_GROUND_RocketON);
        gcs().send_text(MAV_SEVERITY_INFO, "Rocket ground ON");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Rocket ground ON");
    }
}

void Plane::HB1_msg_mission2apm_EngineSTART_handle() {
    if (!arming.is_armed()) {
        // check if need to re send start cmd
        if (HB1_Power_running()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine is running");
        } else {
            HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineSTART_PRE, true);
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground start");
    }
}

void Plane::HB1_msg_mission2apm_EngineOFF_handle() {
    HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineLOW, true);
}

void Plane::HB1_msg_mission2apm_ParachuteON_handle() {
    plane.HB1_status_set_HB_Power_Action(plane.HB1_PowerAction_ParachuteON);
}

void Plane::HB1_msg_mission2apm_EngineFULL_handle() {
    if (arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground start");
        return;
    } 
    if (HB1_Power.state == HB1_PowerAction_None) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is not running");
        return;
    }
    if (HB1_Power.state == HB1_PowerAction_GROUND_EngineSTART) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is pulling up");
        return;
    }

    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineFULL, true);
}

void Plane::HB1_msg_mission2apm_EngineMID_handle() {
    if (arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm first! for Engine ground start");
        return;
    } 
    if (HB1_Power.state == HB1_PowerAction_None) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is not running");
        return;
    }
    if (HB1_Power.state == HB1_PowerAction_GROUND_EngineSTART) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine is booting");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Engine ground mid");
    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineMID, true);
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
