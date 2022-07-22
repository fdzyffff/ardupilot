#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_angle::init(bool ignore_checks)
{
    if (copter.Utarget.is_active()) {
        // initialise position and desired velocity
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }
        copter.g2.user_parameters.Ucam_pid.reset_I();
        copter.g2.user_parameters.Ucam_pid.reset_filter();
        copter.Upayload.set_state(UPayload::payload_arm);
        return true;
    }
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_angle::run()
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    float target_roll_ang = copter.Utarget.get_target_roll_angle();
    float target_pitch_rate = copter.Utarget.get_target_pitch_rate();

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) > copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MIN(0.0f,target_pitch_rate);
    } else if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MAX(0.0f,target_pitch_rate);
    }

    // get target yaw rate
    float target_yaw_rate = copter.Utarget.get_target_yaw_rate();

    // get target climb rate
    float target_climb_rate = my_get_target_climb_rate();

    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {
    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    default:
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

    if (copter.ins.get_accel_peak_hold_neg_x() < -(20.0f)) {
        copter.Upayload.set_state(UPayload::payload_fire);
    }

}

float ModeAttack_angle::my_get_target_climb_rate() {
    float current_angle_deg = copter.Utarget.get_current_angle_deg();
    float target_angle_deg = copter.g2.user_parameters.fly_attack_angle*0.01f;
    float climb_rate_factor = copter.g2.user_parameters.fly_climb_factor;
    float pitch_scalar = copter.g2.user_parameters.fly_pitch_scalar*constrain_float(fabsf(degrees(copter.ahrs_view->pitch)*100.f/copter.g2.user_parameters.fly_pitch_limit.get()), 0.3f, 1.0f);

    float final_climb_rate = get_pilot_speed_dn()*(target_angle_deg - current_angle_deg)* 0.1f * climb_rate_factor * pitch_scalar;
    //if (final_climb_rate > 0.33f*g.pilot_speed_up) {final_climb_rate = 0.33f*g.pilot_speed_up;}
    return constrain_float(final_climb_rate, -get_pilot_speed_dn(), 0.33f*g.pilot_speed_up);
}
