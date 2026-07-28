#include "Copter.h"

bool ModeAttack::init(bool ignore_checks)
{
    if (!copter.uattack.is_active()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: target unavailable");
        return false;
    }

    throttle_out = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    target_lost_start_ms = 0;
    return true;
}

void ModeAttack::run()
{
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        throttle_out = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        throttle_out = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    Vector3f target_rate_b_dps;
    if (copter.uattack.is_active()) {
        target_rate_b_dps = copter.uattack.get_target_rate_b_dps();
        target_lost_start_ms = 0;
    } else {
        target_rate_b_dps.zero();

        const uint32_t now_ms = AP_HAL::millis();
        if (target_lost_start_ms == 0) {
            target_lost_start_ms = now_ms;
        } else if (now_ms - target_lost_start_ms >= 2000U) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: target lost, RTL");
            if (!copter.set_mode(Mode::Number::RTL, ModeReason::FAILSAFE)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: RTL failed, LAND");
                copter.set_mode(Mode::Number::LAND, ModeReason::FAILSAFE);
            }
            return;
        }
    }

    const Vector3f target_rate_b_rads(radians(target_rate_b_dps.x),
                                      radians(target_rate_b_dps.y),
                                      radians(target_rate_b_dps.z));
    attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(target_rate_b_rads.x,
                                                           target_rate_b_rads.y,
                                                           target_rate_b_rads.z);

    const float throttle_target = constrain_float(copter.uattack.get_throttle(), 0.0f, 1.0f);
    const float throttle_delta_max = MAX(copter.uattack.get_throttle_rate(), 0.0f) * G_Dt;
    throttle_out += constrain_float(throttle_target - throttle_out,
                                    -throttle_delta_max,
                                    throttle_delta_max);
    throttle_out = constrain_float(throttle_out, 0.0f, 1.0f);

    attitude_control->set_throttle_out(throttle_out, false, copter.g.throttle_filt);
}