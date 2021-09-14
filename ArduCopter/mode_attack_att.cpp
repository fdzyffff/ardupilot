#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_att::init(bool ignore_checks)
{
    if (copter.Ucam.is_active()) {
        return true;
    }
    copter.g2.user_parameters.Ucam_pid.reset_I();
    copter.g2.user_parameters.Ucam_pid.reset_filter();
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_att::run()
{
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    float target_roll_ang = copter.Ucam.get_target_roll_angle();
    float target_pitch_rate = copter.Ucam.get_target_pitch_rate();

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) > copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MIN(0.0f,target_pitch_rate);
    } else if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MAX(0.0f,target_pitch_rate);
    }
    // get target yaw rate
    float target_yaw_rate = copter.Ucam.get_target_yaw_rate();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(my_get_throttle_boosted(motors->get_throttle_hover()*1.1f),
                                       true,
                                       g.throttle_filt);
}

float ModeAttack_att::my_get_throttle_boosted(float throttle_in)
{
    float current_angle_deg = copter.Ucam.get_current_angle_deg();
    float target_angle_deg = MAX(copter.g2.user_parameters.fly_attack_angle*0.01f, 5.0f);

    float norm_input = constrain_float((target_angle_deg - current_angle_deg)/target_angle_deg, -1.0f, 1.0f);

    float throttle_out = throttle_in + copter.g2.user_parameters.Thr_pid.update_all(0.0f, norm_input, false);
    return throttle_out;
}