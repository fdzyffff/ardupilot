#include "Copter.h"

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool ModeLockon::init(bool ignore_checks)
{
    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);

    // initialise position controller
    pos_control->init_xy_controller();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode lockon");

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeLockon::run()
{
    float target_yaw_rate = 0.0f;
    // get pilot's desired yaw rate
    if (copter.Ucam.is_active()) {
        target_yaw_rate = copter.Ucam.get_target_yaw_rate();
    } else {
        target_yaw_rate = copter.Ugcs.get_lockon_yaw_rate();
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // use position controller to stop
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);

    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();

    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

}


uint32_t ModeLockon::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLockon::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
