#include "mode.h"
#include "Plane.h"

bool ModeGG::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;

    _track_covered = 0.0f;
    _track_dist = 0.0f;
    if (!plane.HB1_status_noGPS_check()) {
        _HB1_attack_prev_WP_loc = plane.current_loc;
        _HB1_attack_next_WP_loc = plane.current_loc;
        _HB1_attack_next_WP_loc = plane.HB1_attack_cmd.content.location;
        // convert relative alt to absolute alt
        if (_HB1_attack_next_WP_loc.relative_alt) {
            _HB1_attack_next_WP_loc.relative_alt = false;
            _HB1_attack_next_WP_loc.alt += plane.home.alt;
        }
        plane.prev_WP_loc = _HB1_attack_prev_WP_loc;
        plane.next_WP_loc = _HB1_attack_next_WP_loc;
        //_HB1_attack_next_WP_loc.lng = plane.HB1_attack_cmd.content.location.lng;
        //_HB1_attack_next_WP_loc.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
        plane.setup_glide_slope();
        //plane.setup_turn_angle();
        plane.auto_state.next_turn_angle = 0;
        //plane.set_target_altitude_location(_HB1_attack_next_WP_loc);
        // disable crosstrack, head directly to the point
        plane.auto_state.crosstrack = false;

        // reset loiter start time.
        plane.loiter.start_time_ms = 0;

        // start in non-VTOL mode
        plane.auto_state.vtol_loiter = false;
        // // zero out our loiter vals to watch for missed waypoints
        plane.set_target_altitude_location(plane.next_WP_loc);

        plane.set_offset_altitude_location(plane.next_WP_loc);
        _dir_unit = _HB1_attack_prev_WP_loc.get_distance_NE(_HB1_attack_next_WP_loc);
        _track_dist = _dir_unit.length() * 100.f;
        _dir_unit.normalize();

        plane.TECS_controller.set_attack_param(plane.g2.hb1_gg_tecs_time_const.get(), plane.g2.hb1_gg_tecs_spdweight.get(), plane.g2.hb1_gg_tecs_pitch_damp.get());
        plane.aparm.airspeed_cruise_cm.set(plane.g2.hb1_gg_spd.get()*100.f);

        gcs().send_text(MAV_SEVERITY_INFO, "_track_dist :%0.1f, (%0.2f, %0.2f)", _track_dist, _dir_unit.x, _dir_unit.y);
        set_HB1_GG_state(HB1_GG_STEP1);
    } else {
        set_HB1_GG_state(HB1_GG_STEP2);
    }
    return true;
}

void ModeGG::update()
{
    static int16_t print_counter = 0;
    print_counter++;
    if (print_counter > 400) {
        print_counter = 0;
    }

    float final_gg_sec = constrain_float(plane.g2.hb1_gg_sec, 0.5f, 10.f);
    // set nav_roll and nav_pitch using sticks
    float final_speed_cm = 100.f* (plane.g2.hb1_follow_speed + plane.g2.hb1_follow_speed_range);
    switch(HB1_GG_state) {
        case HB1_GG_STEP1:
        {
            if (plane.HB1_status_noGPS_check()) {
                set_HB1_GG_state(HB1_GG_STEP2);
            }
            // use to update wp_distance in navigation.
            plane.prev_WP_loc = _HB1_attack_prev_WP_loc;
            plane.next_WP_loc = _HB1_attack_next_WP_loc;
            // update waypoint controller
            plane.calc_nav_roll();
            plane.calc_throttle();
            plane.calc_nav_pitch();

            _track_covered = _dir_unit * (_HB1_attack_prev_WP_loc.get_distance_NE(plane.current_loc));
            _track_covered *= 100.f;










            float delta_xy_cm = 100.f * plane.current_loc.get_distance(plane.HB1_attack_cmd.content.location);
            static float delta_z_cm = 0.0f;
            int32_t tmp_curr_alt = 0;
            int32_t tmp_attack_alt = 0;
            if (plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, tmp_curr_alt) && plane.HB1_attack_cmd.content.location.get_alt_cm(Location::AltFrame::ABOVE_HOME, tmp_attack_alt)) {
                delta_z_cm = (float)tmp_curr_alt -(float)tmp_attack_alt;
            }
            delta_z_cm = MAX(delta_z_cm, 10.0f);
            int16_t current_cd = plane.ahrs.yaw_sensor;//plane.gps.ground_course_cd();
            int16_t target_cd = plane.current_loc.get_bearing_to(plane.HB1_attack_cmd.content.location);
            float delta_yaw = (float)constrain_int16(wrap_180_cd(target_cd - current_cd), -4500,4500) / 4500.f;
            float target_roll = 0.0f;
            if (delta_yaw < 0.0f) {
                target_roll = 0.8f * delta_yaw * delta_yaw + 1.8f * delta_yaw;
                target_roll *= 7000.f;
                //plane.nav_roll_cd = constrain_int16((int16_t)target_roll, -7000, 7000);
            } else {
                target_roll = -0.8f * delta_yaw * delta_yaw + 1.8f * delta_yaw;
                target_roll *= 7000.f;
                //plane.nav_roll_cd = constrain_int16((int16_t)target_roll, -7000, 7000);
            }
            //plane.nav_roll_cd = 0;


            float target_pitch = -100.0f * ToDeg(atan2f(delta_z_cm, delta_xy_cm));
            // if (delta_z_cm < 1000.f) {
            //     plane.nav_pitch_cd = constrain_int16((int16_t)target_pitch, -8500, -6500);
            // } else {
            float tmp_pitch = MIN(plane.nav_pitch_cd, constrain_int16((int16_t)target_pitch, -8500, 500));
            tmp_pitch += 1;
            //plane.nav_pitch_cd = MIN(plane.nav_pitch_cd, constrain_int16((int16_t)target_pitch, -8500, 500));
            // }
            //plane.aparm.airspeed_cruise_cm.set(final_speed_cm);


















            // once track covered, go final stage
            if ((_track_dist - _track_covered) < -(MAX(0.1f,final_gg_sec) * final_speed_cm)) {
                set_HB1_GG_state(HB1_GG_STEP2);
                gcs().send_text(MAV_SEVERITY_INFO, "Force Final");
            }
            if (print_counter%133 == 0) {                
                gcs().send_text(MAV_SEVERITY_INFO, "_track_dist ; _track_rest : %0.2f, %0.2f", _track_dist, _track_dist - _track_covered);
                gcs().send_text(MAV_SEVERITY_INFO, "target_roll : %0.2f", target_roll);
                gcs().send_text(MAV_SEVERITY_INFO, "c_cd %d, t_cd: %d, d_cd: %d", current_cd, target_cd, plane.nav_roll_cd);
                gcs().send_text(MAV_SEVERITY_INFO, "P_cd : %d", plane.nav_pitch_cd);
            }
            break;
        }
        case HB1_GG_STEP2:
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = -8500;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            plane.update_load_factor();
            plane.adjust_nav_pitch_throttle();
            break;
    }

    if (plane.arming.is_armed()) {
        if (plane.ins.get_accel_peak_hold_neg_x() < -(20.0f)){
// #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//             plane.arming.disarm();
//             gcs().send_text(MAV_SEVERITY_INFO, "Disarm motors [SITL]");
//             gcs().send_text(MAV_SEVERITY_INFO, "Hit at (%0.2f) M", plane.current_loc.get_distance(_HB1_attack_next_WP_loc));

// #else
            if (print_counter%400 == 0) {  
                gcs().send_text(MAV_SEVERITY_INFO, "Hit at (%0.2f) M", plane.current_loc.get_distance(_HB1_attack_next_WP_loc));
            }
// #endif
        }
    }
}

void ModeGG::set_HB1_GG_state(HB1_GG_t action) {
    HB1_GG_state = action;
    switch (action) {
        case HB1_GG_STEP1:
            gcs().send_text(MAV_SEVERITY_INFO, "GG step1");
            break;
        case HB1_GG_STEP2:
            plane.aparm.pitch_limit_min_cd.set(-8500);
            //plane.aparm.roll_limit_cd.set(2000);
            gcs().send_text(MAV_SEVERITY_INFO, "GG step2");
            break;
    }
}