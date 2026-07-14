#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQGuided::_enter()
{
    // initialise loiter
    Location loc{plane.current_loc};

    loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());

    plane.set_guided_WP(loc);
    quadplane.wp_nav->set_wp_destination_loc(plane.next_WP_loc);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    is_takeoff = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Vtol Q Guided");

    return true;
}

void ModeQGuided::_exit()
{
    ;
}

void ModeQGuided::navigate()
{
    // reset takeoff if we aren't armed
    if (is_takeoff) {
        if (!plane.arming.is_armed_and_safety_off()) {
            plane.quadplane.do_vtol_takeoff(_tkoff_cmd);
        }
    }
}

void ModeQGuided::update()
{

}

// run quadplane loiter controller
void ModeQGuided::run()
{
    if (is_takeoff) {
        takeoff_run();
    } else {
        wp_run();
    }
}

void ModeQGuided::takeoff_run()
{
    plane.quadplane.takeoff_controller();
    return;
}

void ModeQGuided::wp_run()
{
    if (quadplane.assist.check_VTOL_recovery()) {
        // use QHover to recover from extreme attitudes, this allows
        // for the fixed wing controller to handle the recovery
        plane.mode_qhover.run();
        return;
    }

    const uint32_t now = AP_HAL::millis();

    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    if (!quadplane.motors->armed()) {
        // initialise loiter
        Location loc{plane.current_loc};

        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                               plane.quadplane.stopping_distance());

        plane.set_guided_WP(loc);
        quadplane.wp_nav->set_wp_destination_loc(plane.next_WP_loc);

        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
        pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

        is_takeoff = false;
        return;
    }

    quadplane.setup_target_position();

    /*
      this is full copter control of auto flight
    */
    if (!quadplane.pos_control->is_active_xy()) {
        quadplane.pos_control->init_xy_controller();
    }


    if (quadplane.should_relax()) {
        ;
    }

    // run wpnav controller
    quadplane.wp_nav->update_wpnav();

    // nav roll and pitch are controller by waypoint controller
    plane.nav_roll_cd = quadplane.wp_nav->get_roll();
    plane.nav_pitch_cd = quadplane.wp_nav->get_pitch();

    quadplane.assign_tilt_to_fwd_thr();

    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        quadplane.pos_control->set_externally_limited_xy();
    }

    // call attitude controller
    quadplane.disable_yaw_rate_time_constant();
    quadplane.attitude_control->input_euler_angle_roll_pitch_yaw(plane.nav_roll_cd,
                                                       plane.nav_pitch_cd,
                                                       _yaw_cd,
                                                       true);

    // climb based on altitude error
    quadplane.run_z_controller();
}

void ModeQGuided::do_takeoff(float alt_m, float yaw_cd)
{
    if (is_takeoff) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already takeoff, is_takeoff = true");
        return;
    }

    if (plane.quadplane.is_flying()) {
        const float thr = plane.quadplane.motors->get_throttle();
        const bool thr_lower = plane.quadplane.motors->limit.throttle_lower;
        const bool in_trans = plane.quadplane.tailsitter.in_vtol_transition();
        gcs().send_text(MAV_SEVERITY_INFO, "do_takeoff: is_flying=true, thr=%.3f, thr_lower=%d, in_vtol_trans=%d, ",
                        (float)thr, (int)thr_lower, (int)in_trans);

        if (!in_trans && thr < 0.15f) {
            gcs().send_text(MAV_SEVERITY_INFO, "takeoff with thr %0.3f", (float)thr);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
            return ;
        }
    }

    if (!plane.arming.is_armed_and_safety_off()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Arm first before takeoff");
        return ;
    }

    // set vertical speed and acceleration limits
    _tkoff_cmd.id = MAV_CMD_NAV_TAKEOFF;
    _tkoff_cmd.content.location = plane.current_loc;
    uint32_t target_alt = MAX(alt_m, 1.0f) * 100.f;
    if (_tkoff_cmd.content.location.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        _tkoff_cmd.content.location.set_alt_cm(target_alt, Location::AltFrame::ABOVE_HOME);
    } else {
        return ;
    }

    // reset takeoff if we aren't armed
    plane.quadplane.do_vtol_takeoff(_tkoff_cmd);
    is_takeoff = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Vtol Q Takeoff");
    _yaw_cd = yaw_cd;
}

void ModeQGuided::do_guide(Location &loc_in, float yaw_cd)
{
    const Location &loc = plane.next_WP_loc;
    // const uint32_t now = AP_HAL::millis();
    if (!loc.same_loc_as(loc_in)) {
        plane.next_WP_loc = loc_in;
        quadplane.wp_nav->set_wp_destination_loc(plane.next_WP_loc);
    }

    _yaw_cd = yaw_cd;
    is_takeoff = false;
}

#endif
