#include "Plane.h"

void Plane::HB1_uart_init() {
    HB1_uart_mission.init();
    HB1_uart_mission.get_msg_mission2apm().set_enable();
    HB1_uart_mission.get_msg_mission2cam().set_enable();

    HB1_uart_cam.init();
    HB1_uart_cam.get_msg_cam2mission().set_enable();

    HB1_uart_power.init();
    HB1_uart_power.get_msg_power2apm().set_enable();

    FD1_mav_init();
}

void Plane::HB1_uart_update_50Hz()
{
    if (HB1_uart_mission.initialized()) {
        HB1_uart_mission.read();
    }
    if (HB1_uart_cam.initialized()) {
        HB1_uart_cam.read();
    }
    if (HB1_uart_power.initialized()) {
        HB1_uart_power.read();
    }

    // uart mission
    if (HB1_uart_mission.get_msg_mission2apm()._msg_1.updated) {
        HB1_msg_mission2apm_handle();
        HB1_uart_mission.get_msg_mission2apm()._msg_1.updated = false;
    }

    if (HB1_uart_mission.get_msg_mission2cam()._msg_1.updated) {
        //if (HB1_uart_mission.get_msg_mission2cam()._msg_1.content.msg.uav_id == g.sysid_this_mav) {
            memcpy(HB1_uart_cam.get_msg_mission2cam()._msg_1.content.data, 
                HB1_uart_mission.get_msg_mission2cam()._msg_1.content.data, 
                HB1_uart_mission.get_msg_mission2cam()._msg_1.length*sizeof(uint8_t));
            HB1_uart_cam.get_msg_mission2cam()._msg_1.need_send = true;
        HB1_uart_mission.get_msg_mission2cam()._msg_1.updated = false;
    }

    // uart cam
    if (HB1_uart_cam.get_msg_cam2mission()._msg_1.updated) {
        memcpy(HB1_uart_mission.get_msg_cam2mission()._msg_1.content.data, 
                HB1_uart_cam.get_msg_cam2mission()._msg_1.content.data, 
                HB1_uart_cam.get_msg_cam2mission()._msg_1.length*sizeof(uint8_t));
        HB1_uart_mission.get_msg_cam2mission()._msg_1.need_send = true;
        HB1_uart_cam.get_msg_cam2mission()._msg_1.updated = false;
    }

    // uart power
    if (HB1_uart_power.get_msg_power2apm()._msg_1.updated) {
        HB1_msg_power2apm_handle();
        memcpy(HB1_uart_mission.get_msg_power2apm()._msg_1.content.data, 
                HB1_uart_power.get_msg_power2apm()._msg_1.content.data, 
                HB1_uart_power.get_msg_power2apm()._msg_1.length*sizeof(uint8_t));
        HB1_uart_mission.get_msg_power2apm()._msg_1.need_send = true;
        HB1_uart_power.get_msg_power2apm()._msg_1.updated = false;
    }

    // for test
    if (HB1_test.status != 0) {
        // test_HB1_mission_send_msg();
        //gcs().send_text(MAV_SEVERITY_INFO, "SIM out running");
    }

    HB1_uart_mission.write();
    int16_t _cam_count_trig = 50/constrain_int16(g2.hb1_cam_rate.get(), 1, 50);
    static int16_t _cam_count = 0;
    if (_cam_count >= _cam_count_trig) {
        HB1_uart_cam.write();
        _cam_count = 0;
    }
    _cam_count += 1;

    //FD1_mav_read();
    FD1_mav_send();
}

void Plane::HB1_uart_update_10Hz()
{
    HB1_msg_apm2mission_send();
    HB1_msg_apm2cam_send();

    if (HB1_uart_power.get_msg_apm2power()._msg_1.need_send && HB1_Power.send_counter > 0) {
        if (HB1_Power.send_counter > 1) {
            HB1_Power.send_counter--;
            HB1_uart_power.get_msg_apm2power()._msg_1.need_send = true;
        }
    } else {
        HB1_Power_throttle_update();
    }
    HB1_uart_power.write();
    HB1_uart_print();
}

void Plane::HB1_msg_mission2apm_handle() {
    HB1_Status.last_update_ms = millis();
    //if (HB1_status_noGPS_check()) {return;}
    // pack up msg
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    // tmp_msg._msg_1.print = false;

    switch (tmp_msg._msg_1.content.msg.remote_index) {
        case 0x63:
            HB1_msg_mission2apm_takeoff_handle();
            break;
        case 0x9C:
            HB1_msg_mission2apm_set_wp_handle();
            // tmp_msg._msg_1.print = true;
            break;
        case 0x66:
            HB1_msg_mission2apm_set_interim_handle();
            // tmp_msg._msg_1.print = true;
            break;
        case 0x33:
            // if (!HB1_status_noGPS_check()) {
                HB1_msg_mission2apm_set_attack_handle();
            // }
            // tmp_msg._msg_1.print = true;
            break;
        // case 0xA3:
        //     if (!HB1_status_noGPS_check()) {
        //         HB1_msg_mission2apm_away_handle();
        //     }
        //     break;
        case 0xE5:
            HB1_Status.grouped = false;
            HB1_msg_mission2apm_attack_handle();
            break;
        case 0x69:
            HB1_Status.grouped = false;
            HB1_msg_mission2apm_preattack_handle(tmp_msg._msg_1.content.msg.remote_cmd.cmd_preattack.time_s);
            break;
        case 0x3A:
            HB1_msg_mission2apm_speed_up_handle();
            break;
        case 0xA7:
            HB1_msg_mission2apm_speed_down_handle();
            break;
        // case 0x35:
        //     to do delete line_index
        //     break;
        case 0xA5:
            HB1_msg_mission2apm_EngineSTART_handle();
            break;
        case 0xC6:
            HB1_msg_mission2apm_EngineOFF_handle();
            break;
        case 0x99:
            HB1_msg_mission2apm_ServoTest_handle();
            break;
        case 0xCC:
            HB1_msg_mission2apm_Disarm_handle();
            break;
        case 0xE7:
            HB1_msg_mission2apm_EngineFULL_handle();
            break;
        case 0xB4:
            HB1_msg_mission2apm_EngineMID_handle();
            break;
        case 0x55:
            HB1_msg_mission2apm_RocketON_handle();
            break;
        case 0x7E:
            HB1_msg_mission2apm_Search_wp_handle();
            break;
        default:
            break;
    }

    if (tmp_msg._msg_1.content.msg.in_group) {
        if (!HB1_status_noGPS_check()) {
            HB1_msg_mission2apm_follow_handle();
        }
        HB1_Status.grouped = true;
    } else {
        if (HB1_Status.grouped) {
            if (!HB1_status_noGPS_check()) {
                HB1_msg_mission2apm_away_handle(tmp_msg);
            }
        }
    }
}

void Plane::HB1_msg_power2apm_handle() {
    HB1_power2apm &tmp_msg = HB1_uart_power.get_msg_power2apm();
    HB1_Power.HB1_engine_fuel = (float)tmp_msg._msg_1.content.msg.FQ340_fuel;
    switch (g2.hb1_power_type.get()) {
        case 0:
            HB1_Power.HB1_engine_rpm.apply((float)tmp_msg._msg_1.content.msg.CYS350_rpm);
            HB1_Power.HB1_engine_temp = 0.1f*(float)tmp_msg._msg_1.content.msg.CYS350_temp;
            break;

        case 10:
            HB1_Power.HB1_engine_rpm.apply((float)tmp_msg._msg_1.content.msg.FQ340_rpm);
            HB1_Power.HB1_engine_temp = (float)tmp_msg._msg_1.content.msg.FQ340_temp1;
            break;

        case 11:
            HB1_Power.HB1_engine_rpm.apply((float)tmp_msg._msg_1.content.msg.FQ340_rpm);
            HB1_Power.HB1_engine_temp = (float)tmp_msg._msg_1.content.msg.FQ340_temp2;
            break;

        case -99:
            HB1_Power.HB1_engine_rpm.apply((float)tmp_msg._msg_1.content.msg.CYS350_rpm);
            HB1_Power.HB1_engine_temp = 0.1f*(float)tmp_msg._msg_1.content.msg.CYS350_temp;
            break;

        case 100:
            HB1_Power.HB1_engine_rpm.reset((float)(millis() - HB1_Power.last_update_ms));
            HB1_Power.HB1_engine_temp = 0.1f*(float)tmp_msg._msg_1.content.msg.CYS350_temp;
            break;

        default:
            HB1_Power.HB1_engine_rpm.apply((float)tmp_msg._msg_1.content.msg.CYS350_rpm);
            HB1_Power.HB1_engine_temp = 0.1f*(float)tmp_msg._msg_1.content.msg.CYS350_temp;
            break;
    }
    HB1_Power.last_update_ms = millis();
}

void Plane::HB1_msg_apm2mission_send() {
    HB1_apm2mission &tmp_msg = HB1_uart_mission.get_msg_apm2mission();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2mission::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2mission::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2mission::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;


    if (HB1_Status.search_wp) {
        HB1_msg_mission2apm_Search_wp_pack();
    } else {
        tmp_msg._msg_1.content.msg.remote_index = 0xFF;
        tmp_msg._msg_1.content.msg.line_index = 0;
        tmp_msg._msg_1.content.msg.point_index = 0;
        tmp_msg._msg_1.content.msg.longitude = (int32_t)((double)current_loc.lng * tmp_msg.SF_LL);
        tmp_msg._msg_1.content.msg.latitude = (int32_t)((double)current_loc.lat * tmp_msg.SF_LL);
        tmp_msg._msg_1.content.msg.alt = (int16_t)(relative_ground_altitude(false) * tmp_msg.SF_ALT);
        tmp_msg._msg_1.content.msg.control_id = 0;
    }

    tmp_msg._msg_1.content.msg.ptich = (int16_t)((float)(ahrs.pitch_sensor/100) * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.roll = (int16_t)((float)(ahrs.roll_sensor/100) * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.yaw = (int16_t)((float)wrap_180_cd(ahrs.yaw_sensor/100) * tmp_msg.SF_ANG);
        float airspeed_measured = 0;
        if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
    tmp_msg._msg_1.content.msg.air_speed = (int16_t)(airspeed_measured * tmp_msg.SF_VEL);
    tmp_msg._msg_1.content.msg.error_code1 = HB1_Power_running();
    tmp_msg._msg_1.content.msg.error_code2 = 0;
    tmp_msg._msg_1.content.msg.rc_code = 0;
    tmp_msg._msg_1.content.msg.target_wp_index = mission.get_current_nav_index();
    tmp_msg._msg_1.content.msg.in_group = (HB1_Status.state == HB1_Mission_Follow);
    tmp_msg._msg_1.content.msg.gspd = (int16_t)(ahrs.groundspeed_vector().length() * 0.01f * tmp_msg.SF_VEL);
    tmp_msg._msg_1.content.msg.gspd_dir = (int16_t)(gps.ground_course_cd()*0.01f * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.mission_state = HB1_status_get_HB_Mission_Action();
    tmp_msg._msg_1.content.msg.control_type = 0;
    tmp_msg._msg_1.content.msg.target_dist = 0;
    tmp_msg._msg_1.content.msg.target_control_id = 0;
    if (plane.control_mode == &plane.mode_auto && arming.is_armed()) {
        tmp_msg._msg_1.content.msg.target_dist = (int16_t)current_loc.get_distance(next_WP_loc);
        tmp_msg._msg_1.content.msg.target_control_id = HIGHBYTE(mission.get_current_nav_cmd().p1);
    }
    tmp_msg._msg_1.content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    };

    tmp_msg._msg_1.print = true;
}

void Plane::HB1_msg_apm2cam_send() {
    HB1_apm2cam &tmp_msg = HB1_uart_cam.get_msg_apm2cam();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2cam::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2cam::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2cam::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-4;
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        tmp_msg._msg_1.content.msg.position_status = 0;
    } else {
        tmp_msg._msg_1.content.msg.position_status = 4;
    }
        uint8_t gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second; uint16_t gps_millis;
        gps.get_BeijingTime(gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second, gps_millis);
    tmp_msg._msg_1.content.msg.gps_month = gps_month;
    tmp_msg._msg_1.content.msg.gps_day = gps_day;
    tmp_msg._msg_1.content.msg.gps_hour = gps_hour;
    tmp_msg._msg_1.content.msg.gps_minute = gps_minute;
    tmp_msg._msg_1.content.msg.gps_second = gps_second;
    tmp_msg._msg_1.content.msg.gps_millis = gps_millis;
    tmp_msg._msg_1.content.msg.gps_millis = gps_millis;
    tmp_msg._msg_1.content.msg.longitude = current_loc.lng;
    tmp_msg._msg_1.content.msg.latitude = current_loc.lat;
    tmp_msg._msg_1.content.msg.gps_alt = (int16_t)(relative_ground_altitude(false) / 20);
    tmp_msg._msg_1.content.msg.ground_spd = (int16_t)(ahrs.groundspeed_vector().length() / 20.0f);
    tmp_msg._msg_1.content.msg.ptich_cd = (int16_t)ahrs.pitch_sensor;
    tmp_msg._msg_1.content.msg.roll_cd = (int16_t)ahrs.roll_sensor;
    tmp_msg._msg_1.content.msg.yaw_cd = (int16_t)ahrs.yaw_sensor;
        float airspeed_measured = 0;
        if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
    tmp_msg._msg_1.content.msg.air_speed = (int16_t)airspeed_measured * 2;
    tmp_msg._msg_1.content.msg.baro_alt = barometer.get_altitude() / 20;
    tmp_msg._msg_1.content.msg.gps_yaw = (uint16_t)gps.ground_course_cd();
    tmp_msg._msg_1.content.msg.gps_nstats = gps.num_sats();
    tmp_msg._msg_1.content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    };

    tmp_msg._msg_1.print = true;
}

void Plane::HB1_msg_apm2power_send() {
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = false;
    tmp_msg._msg_1.content.msg.ctrl_cmd = 0;
    tmp_msg._msg_1.content.msg.thr_value = 0;
    switch (HB1_Power.state) {
        case HB1_PowerAction_RocketON:
        case HB1_PowerAction_GROUND_RocketON:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 8;
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_GROUND_EngineSTART:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 4;
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
        case HB1_PowerAction_EnginePullUP:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 0;
            tmp_msg._msg_1.need_send = false;
            break;
        case HB1_PowerAction_EngineOFF:
        case HB1_PowerAction_GROUND_EngineOFF:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 2;
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_ParachuteON:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 1;
            tmp_msg._msg_1.need_send = true;
            break;
        default:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 0;
            tmp_msg._msg_1.need_send = false;
            break;
    }
    if (!tmp_msg._msg_1.need_send) {return;}

    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.print = true;
    return;
}

void Plane::HB1_uart_print(){
    // @Description:  bit8: all 
    //                bit7: apm2cam
    //                bit6: apm2mission
    //                bit5: apm2power
    //                bit4: cam2mission
    //                bit3: mission2apm
    //                bit2: mission2cam
    //                bit1: power2apm 

    if (HB1_uart_power.get_msg_power2apm()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<0)) {
            HB1_uart_power.get_msg_power2apm().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "power2apm :");
            for (int8_t i = 0; i < HB1_uart_power.get_msg_power2apm()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_power.get_msg_power2apm()._msg_1.content.data[i]);
            }
            HB1_uart_power.get_msg_power2apm().swap_message();
        }
        HB1_uart_power.get_msg_power2apm()._msg_1.print = false;
    }

    if (HB1_uart_mission.get_msg_mission2cam()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<1)) {
            HB1_uart_mission.get_msg_mission2cam().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "mission2cam :");
            for (int8_t i = 0; i < HB1_uart_mission.get_msg_mission2cam()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_mission.get_msg_mission2cam()._msg_1.content.data[i]);
            }
            HB1_uart_mission.get_msg_mission2cam().swap_message();
        }
        HB1_uart_mission.get_msg_mission2cam()._msg_1.print = false;
    }

    if (HB1_uart_mission.get_msg_mission2apm()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<2)) {
            HB1_uart_mission.get_msg_mission2apm().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "mission2apm :");
            for (int8_t i = 0; i < HB1_uart_mission.get_msg_mission2apm()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_mission.get_msg_mission2apm()._msg_1.content.data[i]);
            }
            HB1_uart_mission.get_msg_mission2apm().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "apm_deltaX: %0.2f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.apm_deltaX);
            gcs().send_text(MAV_SEVERITY_INFO, "apm_deltaY: %0.2f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.apm_deltaY);
            gcs().send_text(MAV_SEVERITY_INFO, "apm_deltaZ: %0.2f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.apm_deltaZ);
            gcs().send_text(MAV_SEVERITY_INFO, "leader_lng: %0.6f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.leader_lng/HB1_uart_mission.get_msg_mission2apm().SF_LL);
            gcs().send_text(MAV_SEVERITY_INFO, "leader_lat: %0.6f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.leader_lat/HB1_uart_mission.get_msg_mission2apm().SF_LL);
            gcs().send_text(MAV_SEVERITY_INFO, "leader_alt: %0.2f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.leader_alt/HB1_uart_mission.get_msg_mission2apm().SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "leader_dir: %0.2f", (double)HB1_uart_mission.get_msg_mission2apm()._msg_1.content.msg.leader_dir/HB1_uart_mission.get_msg_mission2apm().SF_ANG);
        }
        HB1_uart_mission.get_msg_mission2apm()._msg_1.print = false;

    }

    if (HB1_uart_cam.get_msg_cam2mission()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<3)) {
            HB1_uart_cam.get_msg_cam2mission().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "cam2mission :");
            for (int8_t i = 0; i < HB1_uart_cam.get_msg_cam2mission()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_cam.get_msg_cam2mission()._msg_1.content.data[i]);
            }
            HB1_uart_cam.get_msg_cam2mission().swap_message();
        }
        HB1_uart_cam.get_msg_cam2mission()._msg_1.print = false;
    }

    if (HB1_uart_power.get_msg_apm2power()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<4)) {
            HB1_uart_power.get_msg_apm2power().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "apm2power :");
            for (int8_t i = 0; i < HB1_uart_power.get_msg_apm2power()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_power.get_msg_apm2power()._msg_1.content.data[i]);
            }
            HB1_uart_power.get_msg_apm2power().swap_message();
        }
        HB1_uart_power.get_msg_apm2power()._msg_1.print = false;
    }

    if (HB1_uart_mission.get_msg_apm2mission()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<5)) {
            HB1_uart_mission.get_msg_apm2mission().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "apm2mission :");
            for (int8_t i = 0; i < HB1_uart_mission.get_msg_apm2mission()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_mission.get_msg_apm2mission()._msg_1.content.data[i]);
            }
            HB1_uart_mission.get_msg_apm2mission().swap_message();
        }
        HB1_uart_mission.get_msg_apm2mission()._msg_1.print = false;
    }

    if (HB1_uart_cam.get_msg_apm2cam()._msg_1.print) {
        if (g2.hb1_msg_print.get() & (1<<6)) {
            HB1_uart_cam.get_msg_apm2cam().swap_message();
            gcs().send_text(MAV_SEVERITY_INFO, "apm2cam :");
            for (int8_t i = 0; i < HB1_uart_cam.get_msg_apm2cam()._msg_1.length; i++) {
                gcs().send_text(MAV_SEVERITY_INFO, "  B%d : %X", i+1 , HB1_uart_cam.get_msg_apm2cam()._msg_1.content.data[i]);
            }
            HB1_uart_cam.get_msg_apm2cam().swap_message();
        }
        HB1_uart_cam.get_msg_apm2cam()._msg_1.print = false;
    }

}