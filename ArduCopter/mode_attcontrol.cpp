#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeMyAtt::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Mode MYATT");

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeMyAtt::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    // get pilot's desired yaw rate
    float target_yaw_rate = 0.0f;
    float target_yaw = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = 0.0f;
    if (copter.FD1_hil.healthy) {
        myget_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        target_climb_rate = constrain_float(copter.FD1_hil.ctrl_vel_z_cms, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_yaw_rate = copter.FD1_hil.ctrl_yaw_rate_cd;
        target_yaw = copter.FD1_hil.ctrl_yaw_cd;
    }

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
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    if (copter.FD1_hil.ctrl_mode == 1) {
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
    } else {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    }

    // call z-axis position controller
    pos_control->update_z_controller();

}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void ModeMyAtt::myget_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const
{

    // fetch roll and pitch inputs
    roll_out = copter.FD1_hil.ctrl_roll_cd;
    pitch_out = copter.FD1_hil.ctrl_pitch_cd;

    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

    // roll_out and pitch_out are returned
}
