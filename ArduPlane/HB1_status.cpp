#include "Plane.h"

void Plane::HB1_status_init() {
    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
    HB1_status_set_HB_Mission_Action(HB1_Mission_None);
    SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_parachute_HB1, 0);
    HB1_follow_dir = 0.0f;
    HB1_Status.last_update_ms = 0;
    HB1_Status.mission_complete = false;
    HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
    HB1_Status.already_takeoff = false;
    HB1_Status.grouped = false;
    HB1_Status.search_wp = false;
    HB1_Status.search_line_index = 0;
    HB1_Status.search_id = 1;
    HB1_Status.search_ms = millis();
    HB1_Status.remote_index = 0xFF;

    HB1_Power.HB1_engine_rpm.reset(0.0f);
    HB1_Power.HB1_engine_rpm.set_cutoff_frequency(50.f, 5.f);
    HB1_Power.HB1_engine_temp = 0.0f;
    HB1_Power.HB1_engine_fuel = 0.0f;
    HB1_Power.HB1_engine_startcount = 0;
    HB1_Power.send_counter = 0;

    HB1_GG_final = false;
}

void Plane::HB1_status_update_20Hz() {
    HB1_Power_update();
    HB1_Mission_update();
    HB1_FsAuto_update();
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
            if (HB1_Status.mission_complete || mission.get_current_nav_index() > g2.hb1_num_wp) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Hover);
                HB1_Status.mission_complete = false;
                HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
                AP_Mission::Mission_Command temp_cmd;
                if (mission.get_next_nav_cmd(0, temp_cmd)) {
                    set_target_altitude_location(temp_cmd.content.location);
                }
            }
            break;
        case HB1_Mission_PreLand :
            if (HB1_Status.mission_complete || mission.get_current_nav_index() > (g2.hb1_num_wp + g2.hb1_num_interim)) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Land);
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
            if ( timer > HB1_Status.time_out ){
                if (g2.hb1_pilot_type == 0) {
                    if (g2.hb1_num_land > 0) {
                        HB1_Status.mission_complete = false;
                        HB1_status_set_HB_Mission_Action(HB1_Mission_PreLand);
                    } else {
                        HB1_status_set_HB_Mission_Action(HB1_Mission_Land);
                    }
                }
                if (g2.hb1_pilot_type == 1) {
                    set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
                    HB1_status_set_HB_Mission_Action(HB1_Mission_None);
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineOFF);
                }
            }
            if (HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_FsNoGPS);
            }
            break;
        case HB1_Mission_Hover2 :
            if ((HB1_Status.time_out>0) && (timer > HB1_Status.time_out)) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_Land);
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
                HB1_status_set_HB_Mission_Action(HB1_Mission_Land);
            }
            if (!HB1_status_noGPS_check()) {
                HB1_status_set_HB_Mission_Action(HB1_Mission_WP);
                HB1_Status.time_out = g2.hb1_follow_hover_wp_time;
            }
            break;
        case HB1_Mission_Land :
        case HB1_Mission_FsAuto :
            break;
        default:
            break;
    }
}

void Plane::HB1_status_set_HB_Mission_Action(HB1_Mission_t action, bool Force_set) {
    if ((HB1_Status.state == action) && (!Force_set)) {
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
                if (set_mode(mode_takeoff, MODE_REASON_UNAVAILABLE)) {
                    HB1_Status.state = action;
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_Mission_WP :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_PreLand :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            auto_state.next_wp_crosstrack = false;
            mission.set_current_cmd(g2.hb1_num_wp+MIN(g2.hb1_num_interim,1));
            HB1_Status.state = action;
            break;
        case HB1_Mission_FsGPS :
            set_mode(mode_auto, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_Hover :
            set_mode(mode_loiter, MODE_REASON_UNAVAILABLE);
            next_WP_loc.lat = HB1_lastWP_cmd.content.location.lat;
            next_WP_loc.lng = HB1_lastWP_cmd.content.location.lng;
            // next_WP_loc.alt = plane.current_loc.alt;
            HB1_Status.state = action;
            break;
        case HB1_Mission_Hover2 :
            if (g2.hb1_num_interim == 0) {HB1_interim_cmd.content.location = next_WP_loc;}
            set_mode(mode_loiter, MODE_REASON_UNAVAILABLE);
            if (g2.hb1_num_interim > 0) {
                next_WP_loc = HB1_interim_cmd.content.location;
            }
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
        case HB1_Mission_FsAuto :
            set_mode(mode_fsauto, MODE_REASON_UNAVAILABLE);
            HB1_Status.state = action;
            break;
        case HB1_Mission_Land :
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
        case HB1_Mission_PreLand :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 PreLand");
            break;
        case HB1_Mission_FsGPS :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 FsGPS");
            break;
        case HB1_Mission_Hover :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Hover");
            break;
        case HB1_Mission_Hover2 :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Hover2");
            break;
        case HB1_Mission_Follow :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Follow");
            break;
        case HB1_Mission_FsNoGPS :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 FsNoGPS");
            break;
        case HB1_Mission_FsAuto :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 FsAuto");
            break;
        case HB1_Mission_Land :
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Land");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "HB1 Unknow");
            break;
    }
}

uint8_t Plane::HB1_status_get_HB_Mission_Action() {
    switch (HB1_Status.state) {
        case HB1_Mission_None :
            return 0;
            break;
        case HB1_Mission_Takeoff :
            return 1;
            break;
        case HB1_Mission_Follow :
            return 2;
            break;
        case HB1_Mission_WP :
            return 3;
            break;
        case HB1_Mission_PreLand :
            return 4;
            break;
        case HB1_Mission_Land :
            return 5;
            break;
        case HB1_Mission_Hover :
        case HB1_Mission_Hover2 :
            return 6;
            break;
        case HB1_Mission_FsGPS :
            return 11;
            break;
        case HB1_Mission_FsNoGPS :
            return 12;
            break;
        case HB1_Mission_FsAuto :
            return 13;
            break;
        default:
            return 0;
            break;
    }
}


bool Plane::HB1_status_noGPS_check() {
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return true;
    }
    return false;
}

void Plane::HB1_FsAuto_update() {
    if (g2.hb1_fsauto_time.get() <= 0) {
        return;
    }
    bool ret = false;
    Vector2f vec2f_curr;
    Vector2f vec2f_pre;
    Vector2f vec2f_next;

    if (control_mode != &plane.mode_auto) {
        ret = false;
        return;
    }
    if (!prev_WP_loc.get_vector_xy_from_origin_NE(vec2f_pre)) {
        ret = false;
        return;
    }
    if (!next_WP_loc.get_vector_xy_from_origin_NE(vec2f_next)) {
        ret = false;
        return;
    }
    if (HB1_status_noGPS_check() || !current_loc.get_vector_xy_from_origin_NE(vec2f_curr)) {
        ret = false;
        return;
    }
    Vector2f closest_point = Vector2f::closest_point(vec2f_curr, vec2f_pre, vec2f_next);
    ret = norm(vec2f_curr.x-closest_point.x,vec2f_curr.y-closest_point.y) > 250000.f;

    static uint32_t last_time = millis();
    static bool pre_ret = false;
    if (ret != pre_ret) {
        pre_ret = ret;
        last_time = millis();
        if (ret) {
            gcs().send_text(MAV_SEVERITY_INFO, "Warining! dist %0.0fm away from wp", norm(vec2f_curr.x-closest_point.x,vec2f_curr.y-closest_point.y)*0.01f);
        }
    }
    uint32_t delta_time = millis() - last_time;

    if (ret && ((int32_t)delta_time > g2.hb1_fsauto_time.get()) ) {
        HB1_status_set_HB_Mission_Action(HB1_Mission_FsAuto);
    }
}