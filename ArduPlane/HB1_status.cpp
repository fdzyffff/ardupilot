#include "Plane.h"

void Plane::HB1_status_init() {
    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
    HB1_Power.test_state = HB1_PowerAction_None;
    HB1_status_set_HB_Mission_Action(HB1_Mission_None);
    HB1_Status.num_wp = 0;
    HB1_Status.num_interim = 0;
    HB1_Status.num_attack = 0;
    HB1_follow_dir = 0.0f;
    HB1_Status.last_update_ms = 0;
    HB1_Status.mission_complete = false;
    HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
    HB1_Status.already_takeoff = false;
}

void Plane::HB1_status_update_20Hz() {
    HB1_Power_update();
    HB1_Mission_update();
    //HB1_Servo_status_update();
}

void Plane::HB1_Mission_update() {
    uint32_t timer = millis() - HB1_Status.timer;
    bool mission_alive = (HB1_Status.last_update_ms == 0 || millis() - HB1_Status.last_update_ms < (uint32_t)g2.hb1_follow_fs_time);
    if (control_mode == &mode_fbwa) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_None);
    }

    switch (HB1_Status.state) {
        case HB1_Mission_None :
            break;
        case HB1_Mission_Takeoff :
            // do state check in takeoff mode
            break;
        case HB1_Mission_WP :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
                HB1_Status.mission_complete = false;
                HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_Attack :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
                HB1_Status.mission_complete = false;
                HB1_Status.time_out = g2.hb1_follow_hover_attack_time;
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_FsGPS :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
                HB1_Status.mission_complete = false;
                HB1_Status.time_out = g2.hb1_follow_hover_attack_time;
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_Hover :
            if (timer > HB1_Status.time_out) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_GG);
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_Follow :
            HB1_update_follow();
            if (!mission_alive) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsGPS);
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_FsNoGPS :
            if (timer > (uint32_t)g2.hb1_follow_nogps_time) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_GG);
            }
            if (!HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_WP);
                HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
            }
            break;
        case HB1_Mission_GG :
            break;
        default:
            break;
    }
}

void Plane::HB1_status_set_HB_Mission_Action(HB1_Mission_t action) {
    if (HB1_Status.state == action) {
        return;
    }
    uint32_t tnow = millis();
    HB1_Status.timer = tnow;
    switch (action) {
        case HB1_Mission_None :
            HB1_Status.state = action;
            break;
        case HB1_Mission_Takeoff :
            if (arming.arm(AP_Arming::Method::MAVLINK, true) || arming.is_armed()) {
                set_mode(mode_takeoff, MODE_REASON_UNAVAILABLE);
                HB1_Status.state = action;
            }
            break;
        case HB1_Mission_WP :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            mission.set_current_cmd(1);
            HB1_Status.state = action;
            break;
        case HB1_Mission_Attack :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            mission.set_current_cmd(HB1_Status.num_wp+1);
            HB1_Status.state = action;
            break;
        case HB1_Mission_FsGPS :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_Hover :
            set_mode(mode_loiter, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_Follow :
            set_mode(mode_guided, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_FsNoGPS :
            set_mode(mode_fbwb, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_GG :
            set_mode(mode_gg, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        default:
            break;
    }


    switch (HB1_Status.state) {
        case HB1_Mission_None :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 None");
            break;
        case HB1_Mission_Takeoff :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Takeoff");
            break;
        case HB1_Mission_WP :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 WP");
            break;
        case HB1_Mission_Attack :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Attack");
            break;
        case HB1_Mission_FsGPS :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 FsGPS");
            break;
        case HB1_Mission_Hover :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Hover");
            break;
        case HB1_Mission_Follow :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Follow");
            break;
        case HB1_Mission_FsNoGPS :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 FsNoGPS");
            break;
        case HB1_Mission_GG :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 GG");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Unknow");
            break;
    }
}

bool Plane::HB1_status_noGPS_check() {
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return true;
    }
    return false;
}