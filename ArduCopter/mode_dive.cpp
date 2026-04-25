#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */
bool ModeDive::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // perform minimal arming checks
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        gcs().send_text(MAV_SEVERITY_INFO, "arm check fail");
        return false;
    }

    // do not enter the mode if sticks are not centered or throttle is not at zero
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())
        || !is_zero(channel_throttle->norm_input_dz())) {
        gcs().send_text(MAV_SEVERITY_INFO, "arm check rc fail %f, %f, %f, %f", channel_pitch->norm_input_dz(), channel_roll->norm_input_dz(), channel_yaw->norm_input_dz(), channel_throttle->norm_input_dz());
        return false;
    }

    // turn on notify leds
    change_motor_direction(true);
    AP_Notify::flags.esc_calibration = true;

    return true;
}


bool ModeDive::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeDive::exit()
{
    change_motor_direction(false);

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}

void ModeDive::change_motor_direction(bool inverse) 
{
    ;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeDive::run()
{
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
}

void ModeDive::output_to_motors()
{
    // check if motor are allowed to spin
    const bool allow_output = motors->armed() && motors->get_interlock() && (!copter.failsafe.radio && rc().has_ever_seen_rc_input());

    // Get yaw input
    const float pilot_desired_yaw = channel_yaw->norm_input_dz();

    const float pilot_desired_throttle = get_pilot_desired_throttle();

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        // if output aligns with input then use this motor
        if (!allow_output) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        float motors_output = motors->get_roll_factor(i)*pilot_desired_yaw*0.5f + pilot_desired_throttle;
        motors_output = constrain_float(motors_output, 0.0f, 1.0f);
        int16_t pwm = motors->get_pwm_output_min() - (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        motors->rc_write(i, pwm);
    }
}
