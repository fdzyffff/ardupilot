#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    float target_forward = -target_pitch/9000.f;
    float target_lateral = target_roll/9000.f;
    target_roll = 0.0f;
    target_pitch = 0.0f;
    RC_Channel *roll_4x4_ch = rc().find_channel_for_option(RC_Channel::aux_func_t::ROLL_4X4);
    RC_Channel *pitch_4x4_ch = rc().find_channel_for_option(RC_Channel::aux_func_t::PITCH_4X4);
    if ((roll_4x4_ch != nullptr) && (roll_4x4_ch->get_radio_in() > 0)) {
        target_roll = roll_4x4_ch->norm_input_dz()*4500.f;
    }
    if ((pitch_4x4_ch != nullptr) && (pitch_4x4_ch->get_radio_in() > 0)) {
        target_pitch = pitch_4x4_ch->norm_input_dz()*4500.f;
    }
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    float target_down = motors->get_throttle_in();
    Matrix3f tmp_m;
    tmp_m.from_euler(copter.ahrs_view->roll, copter.ahrs_view->pitch, 0.0f);
    Vector3f tmp_input = Vector3f(target_forward, target_lateral, target_down);
    Vector3f tmp_output = tmp_m * tmp_input;
    // target_forward = 0.0f;
    // target_lateral = 0.0f;
    if (motors->armed()) {
        // gcs().send_text(MAV_SEVERITY_INFO, "target_down %0.1f",target_down);
        // gcs().send_text(MAV_SEVERITY_INFO, "%0.1f, %0.1f, %0.1f",tmp_output.x, tmp_output.y, tmp_output.z);
    }
    motors->set_forward(tmp_output.x);
    motors->set_lateral(tmp_output.y);
    attitude_control->set_throttle_out(tmp_output.z, true, g.throttle_filt);
}