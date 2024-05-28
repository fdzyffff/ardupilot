#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // update_attitude_mode();
    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
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

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);

    float target_down = motors->get_throttle_in();

    Matrix3f tmp_m;
    tmp_m.from_euler(copter.ahrs_view->roll, copter.ahrs_view->pitch, 0.0f);
    Vector3f tmp_input = Vector3f(target_forward, target_lateral, target_down);
    Vector3f tmp_output = tmp_m * tmp_input;
    // target_forward = 0.0f;
    // target_lateral = 0.0f;
    // if (motors->armed()) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "target_down %0.1f",target_down);
    //     gcs().send_text(MAV_SEVERITY_INFO, "%0.1f, %0.1f, %0.1f",tmp_output.x, tmp_output.y, tmp_output.z);
    // }
    motors->set_forward(tmp_output.x);
    motors->set_lateral(tmp_output.y);
    attitude_control->set_throttle_out(tmp_output.z, true, g.throttle_filt);
    // gcs().send_text(MAV_SEVERITY_INFO, "%0.2f, %0.2f, %0.2f",target_down, tmp_output.z, motors->get_throttle());
}
