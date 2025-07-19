#include "Copter.h"

// initialise follow mode
bool ModeAttack::init(const bool ignore_checks)
{
    if (!copter.uattack.is_active()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No target");
    }

    // copter.uattack.attack_roll_pid.set_integrator(0.0f);
    gcs().send_text(MAV_SEVERITY_WARNING, "Throttle I to %0.2f", get_pilot_desired_throttle());

    copter.uattack.start();
    return true;
}

void ModeAttack::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll = copter.uattack.get_target_roll_angle()*100.f; //cd
    float target_pitch_rate = copter.uattack.get_target_pitch_rate()*100.f; //cd/s
    float target_yaw_rate = copter.uattack.get_target_yaw_rate()*100.f; //cd/s

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

    // float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        // pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        // pilot_desired_throttle = 0.0f;
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

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll, target_pitch_rate, target_yaw_rate);

    float target_throttle = copter.uattack._attack_throttle;
    target_throttle = constrain_float(target_throttle, 0.05f, 1.0f);

    // output pilot's throttle
    attitude_control->set_throttle_out(target_throttle, false, g.throttle_filt);
}

void ModeAttack::exit()
{
    copter.uattack.stop();
}