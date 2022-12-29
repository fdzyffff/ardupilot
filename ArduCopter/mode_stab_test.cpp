#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeStabTest::init(bool ignore_checks)
{
    if (!motors->armed()) {return false;}
    _theta_cd = 0.0f;
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeStabTest::run()
{
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    if (is_zero(channel_roll->norm_input_dz())) {
        target_roll = my_get_target_roll_angle();
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    if (is_zero(target_yaw_rate)) {
        target_yaw_rate = copter.Ucam.get_target_yaw_rate();
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);


    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
    // // output pilot's throttle
    // attitude_control->set_throttle_out(my_get_throttle_boosted(motors->get_throttle_hover()),
    //                                    false,
    //                                    g.throttle_filt);

}

float ModeStabTest::my_get_target_roll_angle() {
    float kk = constrain_float(copter.g2.user_parameters.atk_kk.get(), 0.1f, 10.0f);
    float target_q_cds = copter.Ucam.get_q_rate_cds();
    _theta_cd = _theta_cd + kk*G_Dt*target_q_cds;
    _theta_cd = constrain_float(_theta_cd, -6000.f, 6000.f); // inside +/- 60
    float target_roll_angle_cd = 100.f*fabsf(degrees(copter.ahrs_view->pitch))*tanf(radians(_theta_cd*0.01f));
    return target_roll_angle_cd;
}

float ModeStabTest::my_get_throttle_boosted(float throttle_in)
{
    // static float throttle_comp = 0.0f;
    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // if (copter.Ucam.is_active() && (tnow - last_update_ms > 20) ) {
    //     float current_angle_deg = copter.Ucam.get_current_angle_deg();
    //     float target_angle_deg = MAX(copter.g2.user_parameters.fly_attack_angle*0.01f, 5.0f);

    //     float norm_input = constrain_float((current_angle_deg - target_angle_deg)/MAX(10.0f, target_angle_deg), -1.0f, 1.0f);
    //     float pitch_scalar = copter.g2.user_parameters.fly_pitch_scalar*constrain_float(fabsf(degrees(copter.ahrs_view->pitch)*100.f/copter.g2.user_parameters.fly_pitch_limit.get()), 0.3f, 1.0f);

    //     throttle_comp = copter.g2.user_parameters.Thr_pid.update_all(0.0f, norm_input, false);
    //     // throttle_comp *= pitch_scalar;
    //     if (throttle_comp > 0.0) {throttle_comp *= pitch_scalar*constrain_float(copter.g2.user_parameters.atk_thr_up_factor, 0.1f, 2.0f);}
    //     last_update_ms = tnow;
    // } 
    // float throttle_out = throttle_in + throttle_comp;
    // return throttle_out;

    // static int16_t n_count = 0;
    // n_count++;
    float comp_track = 1.0f;
    float current_angle_deg = copter.Ucam.get_current_angle_deg();
    float target_angle_deg = copter.g2.user_parameters.fly_attack_angle*0.01f;
    comp_track = 0.05*(target_angle_deg-current_angle_deg)/MAX(10.f, target_angle_deg);
    comp_track = constrain_float(1.0f + comp_track, 0.95f, 1.05f);

    float costheta = cosf(fabsf(copter.ahrs_view->pitch));
    float comp_thr = 0.0f;
    float t1 = constrain_float(copter.g2.user_parameters.atk_t1.get(), 0.1f, 0.95f);
    float k1 = copter.g2.user_parameters.atk_k1.get();
    float k2 = copter.g2.user_parameters.atk_k2.get();
    float k3 = copter.g2.user_parameters.atk_k3.get();
    if (costheta > t1) {
        comp_thr = k2 + (k1-k2)/(1-t1)*(costheta - t1);
    } else {
        comp_thr = k3 + (k2-k3)/t1*costheta;
    }
    float throttle_out = throttle_in*comp_track*(costheta + comp_thr);

    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = copter.ahrs_view->cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f / constrain_float(cos_tilt, 0.1f, 1.0f);

    // if (n_count > 200) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "step0 %f", throttle_in);
    //     gcs().send_text(MAV_SEVERITY_WARNING, "step1 %f", throttle_out);
    // }
    throttle_out = throttle_out * inverted_factor * boost_factor;
    // if (n_count > 200) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "step2 %f", throttle_out);
    //     gcs().send_text(MAV_SEVERITY_WARNING, "step3 %f, %f, %f", costheta, inverted_factor, boost_factor);
    // }
    throttle_out = constrain_float(throttle_out, 0.0f, 1.0f);

    // if (n_count > 200) {n_count = 0;}

    return throttle_out;
}