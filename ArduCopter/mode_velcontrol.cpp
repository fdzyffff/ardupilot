#include "Copter.h"
#include <utility>

// flowhold_init - initialise flowhold controller
bool ModeMyVel::init(bool ignore_checks)
{

    // initialize vertical speeds and leash lengths
    copter.pos_control->set_max_speed_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_max_accel_z(copter.g.pilot_accel_z);

    // initialise position and desired velocity
    if (!copter.pos_control->is_active_z()) {
        copter.pos_control->set_alt_target_to_current_alt();
        copter.pos_control->set_desired_velocity_z(copter.inertial_nav.get_velocity_z());
    }

    myvel_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), copter.g2.user_parameters.myvel_filter_hz.get());

    copter.g2.user_parameters.myvel_pi_xy.reset_I();
    limited = false;

    copter.g2.user_parameters.myvel_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    gcs().send_text(MAV_SEVERITY_INFO, "Mode MYVEL");

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void ModeMyVel::myvel_vel_to_angle(Vector2f &bf_angles)
{
    // filter the flow rate
    Vector2f input_ef = myvel_filter.apply(Vector2f(copter.FD1_hil.vel_x_cms, copter.FD1_hil.vel_y_cms));

    input_ef = Vector2f(copter.FD1_hil.ctrl_vel_x_cms, copter.FD1_hil.ctrl_vel_y_cms) - input_ef;
    input_ef.x *= -1.0f;
    input_ef.y *= -1.0f;
    // run PI controller
    copter.g2.user_parameters.myvel_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = copter.g2.user_parameters.myvel_pi_xy.get_p();

    // get I term
    if (limited) {
        // only allow I term to shrink in length
        xy_I = copter.g2.user_parameters.myvel_pi_xy.get_i_shrink();
    } else {
        // normal I term operation
        xy_I = copter.g2.user_parameters.myvel_pi_xy.get_pi();
    }

    ef_output += xy_I;
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles += copter.ahrs.rotate_earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void ModeMyVel::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    copter.pos_control->set_max_speed_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_max_accel_z(copter.g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // check for filter change
    if (!is_equal(myvel_filter.get_cutoff_freq(), copter.g2.user_parameters.myvel_filter_hz.get())) {
        myvel_filter.set_cutoff_frequency(copter.g2.user_parameters.myvel_filter_hz.get());
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = 0.0f;
    float target_yaw = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = 0.0f;
    if (copter.FD1_hil.healthy) {
        target_climb_rate = constrain_float(copter.FD1_hil.ctrl_vel_z_cms, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_yaw_rate = copter.FD1_hil.ctrl_yaw_rate_cd;
        target_yaw = copter.FD1_hil.ctrl_yaw_cd;
    }

    // Flow Hold State Machine Determination
    AltHoldModeState myvel_state = get_alt_hold_state(target_climb_rate);


    // Flow Hold State Machine
    switch (myvel_state) {

    case AltHold_MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->set_yaw_target_to_current_heading();
        copter.pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        copter.g2.user_parameters.myvel_pi_xy.reset_I();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        copter.pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, copter.G_Dt, false);
        copter.pos_control->add_takeoff_climb_rate(takeoff_climb_rate, copter.G_Dt);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

        copter.pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // flowhold attitude target calculations
    Vector2f bf_angles;

    if (copter.FD1_hil.healthy) {
        myvel_vel_to_angle(bf_angles);
    } else {
        bf_angles.x = 0.0f;
        bf_angles.y = 0.0f;
    }
    float angle_max = copter.attitude_control->get_althold_lean_angle_max();
    bf_angles.x = constrain_float(bf_angles.x, -angle_max/2, angle_max/2);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max/2, angle_max/2);

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, copter.aparm.angle_max);
#endif

    // call attitude controller
    if (copter.FD1_hil.ctrl_mode == 1) {
        attitude_control->input_euler_angle_roll_pitch_yaw(bf_angles.x, bf_angles.y, target_yaw, true);
    } else {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate);
    }
    // call z-axis position controller
    pos_control->update_z_controller();
}

