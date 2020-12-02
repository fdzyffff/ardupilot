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
    plane.next_WP_loc.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
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
    return true;
}

void ModeGG::update()
{
    // set nav_roll and nav_pitch using sticks
    float final_speed_cm = 100.f* (plane.g2.hb1_follow_speed + plane.g2.hb1_follow_speed_range);
    float delta_xy_cm = 100.f * plane.current_loc.get_distance(plane.HB1_attack_cmd.content.location);
    int32_t tmp_alt = 0;
    float delta_z_cm = 0.0f;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, tmp_alt)) {
        delta_z_cm = (float)tmp_alt;
    }

    if (!plane.HB1_status_noGPS_check() && (delta_xy_cm > final_speed_cm)) { // GPS is OK
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);

        if (delta_xy_cm > 10.0f * final_speed_cm) {
            plane.calc_nav_roll();
            plane.calc_throttle();
            plane.calc_nav_pitch();
        } else {

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


            float target_ptich = -100.0f * ToDeg(atan2f(delta_z_cm, delta_xy_cm));
            if (delta_z_cm < 1000.f) {
                plane.nav_pitch_cd = constrain_int16((int16_t)target_ptich, -8500, -6500);
            } else {
                plane.nav_pitch_cd = constrain_int16((int16_t)target_ptich, -8500, -2500);
            }
            //gcs().send_text(MAV_SEVERITY_INFO, "P_cd : %d", plane.nav_pitch_cd);
            plane.aparm.airspeed_cruise_cm.set(final_speed_cm);
            plane.update_load_factor();
            plane.adjust_nav_pitch_throttle();
            //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        }
    } else {
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = -8500;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        plane.update_load_factor();
        plane.adjust_nav_pitch_throttle();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (plane.arming.is_armed()) {
        if (plane.ins.get_accel_peak_hold_neg_x() < -(plane.g.crash_accel_threshold)){
            plane.arming.disarm();
            gcs().send_text(MAV_SEVERITY_INFO, "Disarm motors [SITL], (%0.2f) M", plane.current_loc.get_distance(plane.HB1_attack_cmd.content.location));
        }
    }
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
#endif
}
