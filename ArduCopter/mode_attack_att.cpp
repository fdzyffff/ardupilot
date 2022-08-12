#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_att::init(bool ignore_checks)
{
    if (!motors->armed()) {return false;}
    if (copter.Ucam.is_active()) {
        copter.Upayload.set_state(UPayload::payload_arm);
        _fired = false;
        return true;
    }
    copter.g2.user_parameters.Ucam_pid.reset_I();
    copter.g2.user_parameters.Ucam_pid.reset_filter();
    copter.g2.user_parameters.Thr_pid.reset_I();
    copter.g2.user_parameters.Thr_pid.reset_filter();
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
    float target_pitch_rate = MIN(0.0f, copter.Ucam.get_target_pitch_rate());

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) > 0.0f ) {
        target_pitch_rate = MIN(0.0f,target_pitch_rate);
    }
    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MAX(0.0f,target_pitch_rate);
    }

    // get target yaw rate
    float target_yaw_rate = copter.Ucam.get_target_yaw_rate();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(my_get_throttle_boosted(motors->get_throttle_hover()*1.0f),
                                       false,
                                       g.throttle_filt);
    
    if (motors->armed()) {
        if (!_fired && copter.ins.get_accel_peak_hold_neg_x() < -(copter.g2.user_parameters.atk_fire_acc.get())) {
            copter.Upayload.set_state(UPayload::payload_fire);
            _fired = true;
        }
    }

}

float ModeAttack_att::my_get_throttle_boosted(float throttle_in)
{
    static float throttle_comp = 0.0f;
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    if (copter.Ucam.is_active() && (tnow - last_update_ms > 20) ) {
        float current_angle_deg = copter.Ucam.get_current_angle_deg();
        float target_angle_deg = MAX(copter.g2.user_parameters.fly_attack_angle*0.01f, 5.0f);

        float norm_input = constrain_float((current_angle_deg - target_angle_deg)/MAX(10.0f, target_angle_deg), -1.0f, 1.0f);
        float pitch_scalar = copter.g2.user_parameters.fly_pitch_scalar*constrain_float(fabsf(degrees(copter.ahrs_view->pitch)*100.f/copter.g2.user_parameters.fly_pitch_limit.get()), 0.3f, 1.0f);

        throttle_comp = copter.g2.user_parameters.Thr_pid.update_all(0.0f, norm_input, false);
        // throttle_comp *= pitch_scalar;
        if (throttle_comp > 0.0) {throttle_comp *= pitch_scalar*constrain_float(copter.g2.user_parameters.atk_thr_up_factor, 0.1f, 2.0f);}
        last_update_ms = tnow;
    } 
    float throttle_out = throttle_in + throttle_comp;
    return throttle_out;
}