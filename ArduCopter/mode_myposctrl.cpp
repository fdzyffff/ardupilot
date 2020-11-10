#include "Copter.h"

#if MODE_MYPOSCTRL_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeMyposctrl::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    gcs().send_text(MAV_SEVERITY_WARNING,"Customized mode");

    return true;
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeMyposctrl::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

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

    case AltHold_Takeoff:
        // initiate take-off
        //if (!takeoff.running()) {
        //    takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        //}

        // get takeoff adjusted pilot and takeoff climb rates
        //takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        //target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // update altitude target and call position controller
        //pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        //pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        //pos_control->update_z_controller();
        attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
        pos_control->set_alt_target(inertial_nav.get_position().z);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms();
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        //pos_control->update_z_controller();
        attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
        pos_control->set_alt_target(inertial_nav.get_position().z);
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // adjust climb rate using rangefinder
        //target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        //target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        //pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        //pos_control->update_z_controller();
        attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
        pos_control->set_alt_target(inertial_nav.get_position().z);
        break;
    }
}

uint32_t ModeMyposctrl::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeMyposctrl::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
