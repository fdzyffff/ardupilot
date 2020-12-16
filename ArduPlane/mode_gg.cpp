#include "mode.h"
#include "Plane.h"

bool ModeGG::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.prev_WP_loc = plane.current_loc;

    gcs().send_text(MAV_SEVERITY_INFO, "GG wp");
    plane.aparm.pitch_limit_min_cd.set(-8500);
    plane.aparm.roll_limit_cd.set(7000);

    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.HB1_attack_cmd.content.location;
    //plane.next_WP_loc.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
    plane.setup_glide_slope();
    //plane.setup_turn_angle();
    plane.auto_state.next_turn_angle = 0;
    //plane.set_target_altitude_location(plane.next_WP_loc);

    // disable crosstrack, head directly to the point
    plane.auto_state.crosstrack = true;

    // reset loiter start time.
    plane.loiter.start_time_ms = 0;

    // start in non-VTOL mode
    plane.auto_state.vtol_loiter = false;
    plane.HB1_GG_final = false;
    return true;
}

void ModeGG::update()
{
    float final_gg_sec = constrain_float(plane.g2.hb1_gg_sec, 1.5f, 10.f);
    // set nav_roll and nav_pitch using sticks
    float final_speed_cm = 100.f* (plane.g2.hb1_follow_speed + plane.g2.hb1_follow_speed_range);
    float delta_xy_cm = 100.f * plane.current_loc.get_distance(plane.HB1_attack_cmd.content.location);

    if (!plane.HB1_status_noGPS_check() && (delta_xy_cm > final_speed_cm)) { // GPS is OK
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);

        if ((delta_xy_cm > MAX(3.0f,final_gg_sec) * final_speed_cm) && (!plane.HB1_GG_final)) {
            plane.calc_nav_roll();
            plane.calc_throttle();
            plane.calc_nav_pitch();
        } else if ((delta_xy_cm > final_gg_sec * final_speed_cm) && (!plane.HB1_GG_final)) {

            int16_t current_cd = plane.ahrs.yaw_sensor;//plane.gps.ground_course_cd();
            int16_t target_cd = plane.current_loc.get_bearing_to(plane.HB1_attack_cmd.content.location);
            float delta_yaw = (float)constrain_int16(wrap_180_cd(target_cd - current_cd), -4500,4500) / 4500.f;
            float target_roll = 0.0f;
            if (delta_yaw < 0.0f) {
                target_roll = 0.8f * delta_yaw * delta_yaw + 1.8f * delta_yaw;
                target_roll *= 7000.f;
                plane.nav_roll_cd = constrain_int16((int16_t)target_roll, -7000, 7000);
            } else {
                target_roll = -0.8f * delta_yaw * delta_yaw + 1.8f * delta_yaw;
                target_roll *= 7000.f;
                plane.nav_roll_cd = constrain_int16((int16_t)target_roll, -7000, 7000);
            }
            //plane.nav_roll_cd = 0;
            //gcs().send_text(MAV_SEVERITY_INFO, "target_roll : %0.2f", target_roll);
            //gcs().send_text(MAV_SEVERITY_INFO, "c_cd %d, t_cd: %d, d_cd: %d", current_cd, target_cd, plane.nav_roll_cd);

            plane.nav_pitch_cd = -1000;
            //gcs().send_text(MAV_SEVERITY_INFO, "P_cd : %d", plane.nav_pitch_cd);
            plane.aparm.airspeed_cruise_cm.set(final_speed_cm);
            plane.update_load_factor();
            plane.adjust_nav_pitch_throttle();
            //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        } else {
            plane.HB1_GG_final = true;
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = -8500;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            plane.update_load_factor();
            plane.adjust_nav_pitch_throttle();
        }
    } else {
        plane.HB1_GG_final = true;
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = -8500;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        plane.update_load_factor();
        plane.adjust_nav_pitch_throttle();
    }

    if (plane.arming.is_armed()) {
        if (plane.ins.get_accel_peak_hold_neg_x() < -(plane.g.crash_accel_threshold)){
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            plane.arming.disarm();
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
#endif
            gcs().send_text(MAV_SEVERITY_INFO, "Disarm motors [SITL], (%0.2f) M", plane.current_loc.get_distance(plane.HB1_attack_cmd.content.location));
        }
    }
}
