#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack_pos::init(bool ignore_checks)
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
        return true;
    }
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_pos::run()
{
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    // float target_roll_ang = copter.Ucam.get_target_roll_angle();
    float target_roll_ang = my_get_target_roll_angle();
    float target_pitch_ang = -fabsf(copter.g2.user_parameters.fly_pitch_rate.get());

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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_ang, target_pitch_ang, target_yaw_rate);

    // get target climb rate
    float target_climb_rate = my_get_target_climb_rate();

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

    
    if (motors->armed()) {
        if (!_fired && copter.ins.get_accel_peak_hold_neg_x() < -(copter.g2.user_parameters.atk_fire_acc.get())) {
            copter.Upayload.set_state(UPayload::payload_fire);
            _fired = true;
        }
    }
}

float ModeAttack_pos::my_get_target_roll_angle() {
    float kk = constrain_float(copter.g2.user_parameters.atk_kk.get(), 0.1f, 10.0f);
    float target_q_cds = copter.Ucam.get_q_rate_cds();
    _theta_cd = _theta_cd + kk*G_Dt*target_q_cds;
    _theta_cd = constrain_float(_theta_cd, -6000.f, 6000.f); // inside +/- 60
    float target_roll_angle_cd = 100.f*fabsf(degrees(copter.ahrs_view->pitch))*tanf(radians(_theta_cd*0.01f));
    return target_roll_angle_cd;
}

float ModeAttack_pos::my_get_target_climb_rate() {
    float _kP = pos_control->get_pos_z_p().kP();
    float accel_cmss = pos_control->get_max_accel_z_cmss();
    float rate_max = g.pilot_speed_up;
    float rate_min = -get_pilot_speed_dn();
    float delta_alt = 100.f * (copter.Ucam.target_pos_SIM.z - copter.Ucam.current_pos_SIM.z);
    float target_rate = 0.0f;
    if (is_zero(_kP)) {
        if (delta_alt > 0.0f) {
            target_rate = safe_sqrt(2.0f * fabsf(delta_alt) * accel_cmss);
        } else {
            target_rate = -safe_sqrt(2.0f * fabsf(delta_alt) * accel_cmss);
        }
    } else {
        if (delta_alt > 0.0f) {
            target_rate = sqrt_controller(fabsf(delta_alt), _kP, accel_cmss, G_Dt);
        } else {
            target_rate = -sqrt_controller(fabsf(delta_alt), _kP, accel_cmss, G_Dt);
        }
    }
    target_rate = constrain_float(target_rate, rate_min, rate_max);
    return target_rate;
}
