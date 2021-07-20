#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeATTEF3::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeATTEF3::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (target_pitch < -1500.f) {
        target_yaw_rate += User_get_pilot_desired_yaw_rate(target_roll);
    } 

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        if (copter.rangefinder_alt_ok()) {takeoff_stop();}
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_fence_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif
        float ef3_target_alt = constrain_float(copter.g2.user_parameters.EF3_target_alt, 30.0f, 600.0f);

        if (!User_rangefinder_check()) {
            target_climb_rate -= g.pilot_speed_up*0.5f;
        } else {
            if (target_climb_rate > g.pilot_speed_up * 0.9f ) {
                copter.surface_tracking.set_target_alt_cm(ef3_target_alt);
            } else if (target_climb_rate > -get_pilot_speed_dn()*0.9f) {
                target_climb_rate = 0.0f;
                copter.surface_tracking.set_target_alt_cm(ef3_target_alt);
            } else {
                target_climb_rate = MAX(target_climb_rate, -50.f);
            }
        }

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

}

float ModeATTEF3::User_get_pilot_desired_yaw_rate(float target_roll) {
    return copter.g2.user_parameters.EF3_yaw_factor * target_roll;
}

bool ModeATTEF3::User_rangefinder_check() {
    return (copter.inertial_nav.get_altitude() < 50.f || copter.rangefinder_alt_ok());
}
