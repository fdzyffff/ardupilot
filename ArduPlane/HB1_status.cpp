#include "Plane.h"

void Plane::HB1_status_init() {
    HB1_status_set_HB_Power_Action(HB1_PoserAction_None);
    HB1_status_set_HB_Mission_Action(HB1_Mission_None);
    HB1_Status.num_wp = 0;
    HB1_Status.num_interim = 0;
    HB1_Status.num_attack = 0;
    HB1_follow_dir = 0.0f;
}

void Plane::HB1_status_update_20Hz() {
    HB1_Power_update();
    HB1_Mission_update();
}

void Plane::HB1_Mission_update() {
    uint32_t timer = millis() - HB1_Status.timer;
    uint32_t timer_out = 15000;
    if (control_mode == &mode_fbwa) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_None);
    }

    switch (HB1_Status.state) {
        case HB1_Mission_None :
            break;
        case HB1_Mission_Takeoff :
            if (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_NORMAL) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_WP);
            }
        case HB1_Mission_WP :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
            }
            break;
        case HB1_Mission_Attack :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
            }
            break;
        case HB1_Mission_FsGPS :
            if (HB1_Status.mission_complete) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
            }
        case HB1_Mission_Hover :
            if (timer > timer_out) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_GG);
            }
            break;
        case HB1_Mission_Follow :
            HB1_update_follow();
            if (fs_mission_check) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsGPS);
            }
            break;
        case HB1_Mission_FsNoGPS :
            if (timer > timer_out) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_GG);
            }
        case HB1_Mission_GG :
            break;
        default:
            break;
    }
}

void Plane::HB1_status_set_HB_Mission_Action(HB1_Mission_t action) {
    if (HB1_Status.state == action) {
        return;
    } else {
        HB1_Status.state = action;
    }
    uint32_t tnow = millis();
    HB1_Status.timer = tnow;
    switch (HB1_Status.state) {
        case HB1_Mission_None :
            break;
        case HB1_Mission_Takeoff :
            set_mode(plane.mode_takeoff, MODE_REASON_UNAVAILABLE);
            break;
        case HB1_Mission_WP :
            set_mode(plane.mode_auto, MODE_REASON_UNAVAILABLE);
            plane.mission.set_current_cmd(1);
            break;
        case HB1_Mission_Attack :
            set_mode(plane.mode_auto, MODE_REASON_UNAVAILABLE);
            plane.mission.set_current_cmd(HB1_Status.num_wp+1);
            break;
        case HB1_Mission_FsGPS :
            set_mode(plane.mode_auto, MODE_REASON_UNAVAILABLE);
            break;
        case HB1_Mission_Hover :
            set_mode(plane.mode_loiter, MODE_REASON_UNAVAILABLE);
            break;
        case HB1_Mission_Follow :
            set_mode(plane.mode_guided, MODE_REASON_UNAVAILABLE);
            break;
        case HB1_Mission_FsNoGPS :
            set_mode(plane.mode_gg, MODE_REASON_UNAVAILABLE);
            break;
        case HB1_Mission_GG :
            set_mode(plane.mode_gg, MODE_REASON_UNAVAILABLE);
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
