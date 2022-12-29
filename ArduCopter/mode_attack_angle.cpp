#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_angle::init(bool ignore_checks)
{
    if (!motors->armed()) {return false;}
    if (copter.Ucam.is_active()) {
        copter.Upayload.set_state(UPayload::payload_arm);
        _fired = false;
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-450.f, 200.f, g.pilot_accel_z);
        copter.g2.user_parameters.Ucam_pid.reset_I();
        copter.g2.user_parameters.Ucam_pid.reset_filter();
        copter.g2.user_parameters.Roll_pid.reset_I();
        copter.g2.user_parameters.Roll_pid.reset_filter();
        _stage = 1;
        _start_ms = millis();
        _theta_cd = 0.0f;
        _thr = MAX(motors->get_throttle_hover()*0.75f, copter.userhook_FastLoop_throttle_get());
        return true;
    }
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_angle::run()
{
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    // float target_roll_ang = copter.Ucam.get_target_roll_angle();
    float target_roll_ang = my_get_target_roll_angle();
    float target_pitch_rate = copter.Ucam.get_target_pitch_rate();

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) > 0.0f ) {
        target_pitch_rate = MIN(0.0f,target_pitch_rate);
    }

    if ( (degrees(copter.ahrs_view->pitch)*100.f + target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        target_pitch_rate = MAX(0.0f,target_pitch_rate);
    }

    // get target yaw rate
    float target_yaw_rate = copter.Ucam.get_target_yaw_rate();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // reoposition
    if (!copter.failsafe.radio) {
        // get pilot desired lean angles
        float target_roll = 0.0f, target_pitch = 0.0f;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        target_roll_ang += target_roll*0.45f;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);


    if (_stage == 1) {
        // get target climb rate
        float target_climb_rate = 70.f;

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

        // call z-axis position controller
        pos_control->update_z_controller();

        float current_angle_deg = copter.Ucam.get_current_angle_deg();
        float target_angle_deg = copter.g2.user_parameters.fly_attack_angle*0.01f;
        if (degrees(fabsf(copter.ahrs_view->pitch)) > 20.f) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "S2 pitch %0.2f", degrees(fabsf(copter.ahrs_view->pitch)));
            _stage = 2;
        }
        if ((millis()-_start_ms)>2500) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "S2 T");
            _stage = 2;
        }
        if (current_angle_deg>target_angle_deg) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "S2 angle %0.2f", current_angle_deg);
            _stage = 2;
        }
    } else {
        // output pilot's throttle
        attitude_control->set_throttle_out(my_get_throttle_boosted(_thr),
                                       false,
                                       g.throttle_filt);
    }
    
    if (motors->armed()) {
        if (!_fired && copter.ins.get_accel_peak_hold_neg_x() < -(copter.g2.user_parameters.atk_fire_acc.get())) {
            copter.Upayload.set_state(UPayload::payload_fire);
            _fired = true;
        }
    }

}

float ModeAttack_angle::my_get_target_roll_angle() {
    float kk = constrain_float(copter.g2.user_parameters.atk_kk.get(), 0.1f, 10.0f);
    float target_q_cds = copter.Ucam.get_q_rate_cds();
    _theta_cd = _theta_cd + kk*G_Dt*target_q_cds;
    _theta_cd = constrain_float(_theta_cd, -6000.f, 6000.f); // inside +/- 60
    float target_roll_angle_cd = 100.f*fabsf(degrees(copter.ahrs_view->pitch))*tanf(radians(_theta_cd*0.01f));
    return target_roll_angle_cd;
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


float ModeAttack_angle::my_get_throttle_boosted(float throttle_in)
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
    comp_track = 0.1*(target_angle_deg-current_angle_deg)/MAX(10.f, target_angle_deg);
    comp_track = constrain_float(1.0f + comp_track, 1.0f, 1.10f);

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