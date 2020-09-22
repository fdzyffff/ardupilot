#include "Plane.h"

void Plane::HB1_msg_mission2apm_follow_handle() {
    // pack up msg
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
        
    Location loc;
    loc.lat = (int32_t)((double)tmp_msg._msg_1.content.msg.leader_lat/tmp_msg.SF_LL);
    loc.lng = (int32_t)((double)tmp_msg._msg_1.content.msg.leader_lng/tmp_msg.SF_LL);
    loc.set_alt_cm((int32_t)((double)tmp_msg._msg_1.content.msg.leader_alt*100.f/tmp_msg.SF_ALT), Location::AltFrame::ABOVE_HOME);
/*    if (plane.control_mode != &plane.mode_guided) {
        //plane.set_mode(plane.mode_guided, MODE_REASON_GCS_COMMAND);
        return;
    }*/
    HB1_follow_loc = loc;
    HB1_follow_loc.offset(tmp_msg._msg_1.content.msg.apm_deltaX, tmp_msg._msg_1.content.msg.apm_deltaY);
    HB1_follow_loc.alt += (int32_t)tmp_msg._msg_1.content.msg.apm_deltaZ;
    HB1_follow_dir = ((float)tmp_msg._msg_1.content.msg.leader_dir)/tmp_msg.SF_ANG;

    Vector3f tmp_target;
    if (!HB1_follow_loc.get_vector_from_origin_NEU(tmp_target)) {
        tmp_target.zero();
    }
    HB1_status_set_HB_Mission_Action(HB1_Mission_Follow);
}

void Plane::HB1_update_follow()
{
    float target_dist = HB1_follow_loc.get_distance(current_loc);
    float length_cut = 200.0f; //meter
    float vel_length = 0.f;
    if (target_dist > length_cut) {
    	prev_WP_loc = current_loc;
    	guided_WP_loc = HB1_follow_loc;
    	next_WP_loc = guided_WP_loc;
    	auto_state.crosstrack = false;
    	vel_length = length_cut * 0.7f;
    } else {
    	prev_WP_loc = HB1_follow_loc;
    	//prev_WP_loc.offset_bearing(HB1_follow_dir, -100);
    	guided_WP_loc = HB1_follow_loc;
    	guided_WP_loc.offset_bearing(HB1_follow_dir, 200);
    	next_WP_loc = guided_WP_loc;
    	auto_state.crosstrack = true;
    	vel_length = 200.f;
    }
    float delta_dist = (guided_WP_loc.get_distance(current_loc) - vel_length)*100.f;
    float spd_kp = 0.15f;
    float target_spd = g2.hb1_follow_speed*100.f + constrain_float(delta_dist*spd_kp, -g2.hb1_follow_speed_range*100.f, g2.hb1_follow_speed_range*100.f);
    aparm.airspeed_cruise_cm.set(target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "tDist : %0.2f , Dist : %0.2f, V : %0.2f", target_dist, delta_dist, target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "POS X: %0.1f, Y: %0.1f, V:%0.1f", tmp_target.x, tmp_target.y, target_spd);
}

/*void Plane::HB1_update_follow(float HB1_follow_dir)
{
    guided_WP_loc = HB1_follow_loc;

    prev_WP_loc = HB1_follow_loc;//current_loc;
    //prev_WP_loc.offset_bearing(HB1_follow_dir, -100);
    // always look 100m ahead
    guided_WP_loc.offset_bearing(HB1_follow_dir, 200);
    auto_state.crosstrack = false;
    next_WP_loc = guided_WP_loc;

    float delta_dist = (guided_WP_loc.get_distance(current_loc) - 200.f)*100.f;
    float spd_kp = 0.1f;
    float target_spd = 1800.f + constrain_float(delta_dist*spd_kp, -800.f, 1300.f);
    aparm.airspeed_cruise_cm.set(target_spd);
    gcs().send_text(MAV_SEVERITY_INFO, "Dist : %0.2f, V : %0.2f", delta_dist, target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "POS X: %0.1f, Y: %0.1f, V:%0.1f", tmp_target.x, tmp_target.y, target_spd);
}*/

