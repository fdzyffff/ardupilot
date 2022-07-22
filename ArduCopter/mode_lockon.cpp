#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLockon::init(bool ignore_checks)
{
    // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
    loiter_nav->clear_pilot_desired_acceleration();

    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "mode Lock ON");
    return true;
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLockon::run()
{
    float target_yaw_rate = -1500.0f;
    float target_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration(0.0f, 0.0f, G_Dt);

    // get pilot's desired yaw rate
    if (copter.Utarget.is_active()) {
        target_yaw_rate = copter.Utarget.get_target_yaw_rate();
    } else {
        target_yaw_rate = copter.Ugcs.get_lockon_yaw_rate();
    }

    // if (copter.rangefinder_alt_ok() && (float)copter.rangefinder_state.alt_cm < 100.f) {
    //     target_climb_rate = 50.f;
    // }
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);


    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Flying:
    default:
        // if (!copter.Ugcs.is_leader()) {
        //     pos_control->set_xy_target(copter.Ugcs.get_dest_loc_vec().x, copter.Ugcs.get_dest_loc_vec().y);
        // }
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

uint32_t ModeLockon::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLockon::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

