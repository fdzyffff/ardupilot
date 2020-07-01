#include "Plane.h"

void Plane::HB1_update_follow(float target_dir)
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
    	//prev_WP_loc.offset_bearing(target_dir, -100);
    	guided_WP_loc = HB1_follow_loc;
    	guided_WP_loc.offset_bearing(target_dir, 200);
    	next_WP_loc = guided_WP_loc;
    	auto_state.crosstrack = true;
    	vel_length = 200.f;
    }
    float delta_dist = (guided_WP_loc.get_distance(current_loc) - vel_length)*100.f;
    float spd_kp = 0.15f;
    float target_spd = 1800.f + constrain_float(delta_dist*spd_kp, -800.f, 1300.f);
    aparm.airspeed_cruise_cm.set(target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "tDist : %0.2f , Dist : %0.2f, V : %0.2f", target_dist, delta_dist, target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "POS X: %0.1f, Y: %0.1f, V:%0.1f", tmp_target.x, tmp_target.y, target_spd);
}

/*void Plane::HB1_update_follow(float target_dir)
{
    guided_WP_loc = HB1_follow_loc;

    prev_WP_loc = HB1_follow_loc;//current_loc;
    //prev_WP_loc.offset_bearing(target_dir, -100);
    // always look 100m ahead
    guided_WP_loc.offset_bearing(target_dir, 200);
    auto_state.crosstrack = false;
    next_WP_loc = guided_WP_loc;

    float delta_dist = (guided_WP_loc.get_distance(current_loc) - 200.f)*100.f;
    float spd_kp = 0.1f;
    float target_spd = 1800.f + constrain_float(delta_dist*spd_kp, -800.f, 1300.f);
    aparm.airspeed_cruise_cm.set(target_spd);
    gcs().send_text(MAV_SEVERITY_INFO, "Dist : %0.2f, V : %0.2f", delta_dist, target_spd);
    //gcs().send_text(MAV_SEVERITY_INFO, "POS X: %0.1f, Y: %0.1f, V:%0.1f", tmp_target.x, tmp_target.y, target_spd);
}*/
