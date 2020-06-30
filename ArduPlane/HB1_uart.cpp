#include "Plane.h"

void Plane::HB1_uart_init() {
    HB1_uart_mission.init();
    HB1_uart_mission.get_msg_mission2apm().set_enable();
    HB1_uart_mission.get_msg_mission2cam().set_enable();

    HB1_uart_cam.init();
    HB1_uart_cam.get_msg_cam2mission().set_enable();

    HB1_uart_power.init();
    HB1_uart_power.get_msg_power2apm().set_enable();
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
        if (HB1_uart_mission.get_msg_mission2cam()._msg_1.content.msg.uav_id == g.sysid_this_mav) {
            memcpy(HB1_uart_cam.get_msg_mission2cam()._msg_1.content.data, 
                HB1_uart_mission.get_msg_mission2cam()._msg_1.content.data, 
                HB1_uart_mission.get_msg_mission2cam()._msg_1.length*sizeof(uint8_t));
            HB1_uart_cam.get_msg_mission2cam()._msg_1.need_send = true;
        }
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

    HB1_uart_mission.write();
    HB1_uart_cam.write();
    HB1_uart_power.write();
}


void Plane::HB1_uart_update_10Hz()
{
    HB1_msg_apm2mission_send();
    HB1_msg_apm2cam_send();
    HB1_msg_apm2power_send();
}

void Plane::HB1_msg_mission2apm_handle() {
    // pack up msg
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
        
    //tmp_msg._msg_1.content.msg.console_type = 0;
    //tmp_msg._msg_1.content.msg.remote_index = 6;

    Location loc;
    loc.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.leader_lat/tmp_msg.SF_LL);
    loc.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.leader_lng/tmp_msg.SF_LL);
    loc.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.leader_alt/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);
    HB1_follow_loc = loc;
    if (plane.control_mode != &plane.mode_guided) {
        plane.set_mode(plane.mode_guided, MODE_REASON_GCS_COMMAND);
    }
    float delta_dist = loc.get_distance(current_loc)*100.f - 5000.f;
    float spd_kp = 0.2f;
    float target_spd = 1800.f + constrain_float(delta_dist*spd_kp, -800.f, 1300.f);
    aparm.airspeed_cruise_cm.set(target_spd);

    Vector3f tmp_target;
    if (!HB1_follow_loc.get_vector_from_origin_NEU(tmp_target)) {
        tmp_target.zero();
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "Dist %0.1f, V:%0.1f, Z : %0.2f", delta_dist, target_spd, tmp_target.z);
    //gcs().send_text(MAV_SEVERITY_INFO, "POS X: %0.1f, Y: %0.1f, V:%0.1f", tmp_target.x, tmp_target.y, target_spd);
    HB1_update_follow();
}

void Plane::HB1_msg_power2apm_handle() {
    ;
}

void Plane::HB1_msg_apm2mission_send() {
    HB1_apm2mission &tmp_msg = HB1_uart_mission.get_msg_apm2mission();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2mission::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2mission::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2mission::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-3;

    tmp_msg._msg_1.content.msg.longitude = (int32_t)((double)current_loc.lng * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.latitude = (int32_t)((double)current_loc.lat * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.alt = (int16_t)(relative_ground_altitude(false) * tmp_msg.SF_ALT);
    tmp_msg._msg_1.content.msg.ptich = (int16_t)((float)ahrs.pitch_sensor * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.roll = (int16_t)((float)ahrs.roll_sensor * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.yaw = (int16_t)((float)wrap_180_cd(ahrs.yaw_sensor) * tmp_msg.SF_ANG);
        float airspeed_measured = 0;
        if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
    tmp_msg._msg_1.content.msg.air_speed = (int16_t)(airspeed_measured * tmp_msg.SF_VEL);
    tmp_msg._msg_1.content.msg.error_code = 0;
    tmp_msg._msg_1.content.msg.rc_code = 0;
    tmp_msg._msg_1.content.msg.target_wp_index = mission.get_current_nav_index();
    tmp_msg._msg_1.content.msg.console_type = 1;
    tmp_msg._msg_1.content.msg.leader_balt = 0;
    tmp_msg._msg_1.content.msg.leader_ralt = 0;
    tmp_msg._msg_1.content.msg.unused[0] = 0;
    tmp_msg._msg_1.content.msg.unused[1] = 0;
    tmp_msg._msg_1.content.msg.unused[2] = 0;
    tmp_msg._msg_1.content.msg.unused[3] = 0;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    };
}

void Plane::HB1_msg_apm2cam_send() {
    HB1_apm2cam &tmp_msg = HB1_uart_cam.get_msg_apm2cam();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2cam::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2cam::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_apm2cam::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-3;
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
        float gps_yaw = 0;
        float gps_yaw_acc = 0;
        if (!gps.gps_yaw_deg(gps_yaw, gps_yaw_acc)) {gps_yaw = 0.0f; gps_yaw_acc = 0.0f;}
    tmp_msg._msg_1.content.msg.gps_yaw = (uint16_t)wrap_360(gps_yaw) * 10;
    tmp_msg._msg_1.content.msg.gps_nstats = gps.num_sats();
    tmp_msg._msg_1.content.msg.sum_check = 0;
    
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    };
}

void Plane::HB1_msg_apm2power_send() {
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.ctrl_cmd = 0;
    switch (HB_Power_Action) {
        case HB1_PoserAction_None:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 4;
            break;
        case HB1_PoserAction_RocketON:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 1;
            break;
        case HB1_PoserAction_EngineON:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 2;
            break;
        case HB1_PoserAction_EngineOFF:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 4;
            break;
        case HB1_PoserAction_ParachuteON:
            tmp_msg._msg_1.content.msg.ctrl_cmd = 4 + 8;
            break;
        default:
            break;
    }
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    };
}
