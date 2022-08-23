#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_angle::init(bool ignore_checks)
{
    if (!motors->armed()) {return false;}
    if (copter.Ucam.is_active()) {
        // initialise the vertical position controller
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        copter.g2.user_parameters.Ucam_pid.reset_I();
        copter.g2.user_parameters.Ucam_pid.reset_filter();
        _fired = false;
        copter.Upayload.set_state(UPayload::payload_arm);
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-450.f, 200.f, g.pilot_accel_z);
        return true;
    }
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_angle::run()
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    float target_roll_ang = copter.Ucam.get_target_roll_angle();
    float target_pitch_rate = copter.Ucam.get_target_pitch_rate();

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) > 0.0f ) {
        target_pitch_rate = MIN(0.0f,target_pitch_rate);
    }

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MAX(0.0f,target_pitch_rate);
    }

    // get target yaw rate
    float target_yaw_rate = copter.Ucam.get_target_yaw_rate();

    // get target climb rate
    float target_climb_rate = my_get_target_climb_rate();

    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {
    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    default:
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

    if (motors->armed()) {
        if (!_fired && copter.ins.get_accel_peak_hold_neg_x() < -(copter.g2.user_parameters.atk_fire_acc.get())) {
            copter.Upayload.set_state(UPayload::payload_fire);
            _fired = true;
        }
    }

}

float ModeAttack_angle::my_get_target_climb_rate() {
    float current_angle_deg = copter.Ucam.get_current_angle_deg();
    float target_angle_deg = copter.g2.user_parameters.fly_attack_angle*0.01f;
    float climb_rate_factor = copter.g2.user_parameters.fly_climb_factor;
    float pitch_scalar = copter.g2.user_parameters.fly_pitch_scalar*constrain_float(fabsf(degrees(copter.ahrs_view->pitch)*100.f/copter.g2.user_parameters.fly_pitch_limit.get()), 0.3f, 1.0f);
    float norm_input = constrain_float((current_angle_deg - target_angle_deg)/MAX(10.0f, target_angle_deg), -1.0f, 1.0f);

    float final_climb_rate = 450.f * (-norm_input) * climb_rate_factor * pitch_scalar;
    return constrain_float(final_climb_rate, -450.f, 50.f);
}
