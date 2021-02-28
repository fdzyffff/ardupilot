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
    float target_roll_ang, target_pitch_rate;

    my_get_target_angles(target_roll_ang, target_pitch_rate);

    // get target yaw rate
    float target_yaw_rate = my_get_target_yaw_rate();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_euler_rate_pitch_yaw(target_roll_ang, target_pitch_rate, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(my_get_throttle_boosted(motors->get_throttle_hover()),
                                       false,
                                       g.throttle_filt);
}

void ModeAttack_att::my_get_target_angles(float &target_roll_ang, float &target_pitch_rate)
{
    float measurement = (copter.Ucam.get_raw_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y); //-1 to +0.1
    float my_target_pitch_rate = -1.0f*copter.g2.user_parameters.Ucam_pid.update_all(0.5f, measurement, false)*copter.g2.user_parameters.fly_pitch_limit.get();
    if (my_target_pitch_rate > 0.0f) {
        my_target_pitch_rate *= 1.0f;
    }
    if ( (degrees(copter.ahrs_view->pitch)*100.f + my_target_pitch_rate*G_Dt) > copter.g2.user_parameters.fly_pitch_limit.get() ) {
        my_target_pitch_rate = MIN(0.0f,my_target_pitch_rate);
    } else if ( (degrees(copter.ahrs_view->pitch)*100.f + my_target_pitch_rate*G_Dt) < -copter.g2.user_parameters.fly_pitch_limit.get() ) {
        my_target_pitch_rate = MAX(0.0f,my_target_pitch_rate);
    }
    target_pitch_rate = my_target_pitch_rate ;
    target_roll_ang = 0.0f;
}

float ModeAttack_att::my_get_target_yaw_rate() {
    float info_x = copter.Ucam.get_raw_info().x;
    float x_length = copter.g2.user_parameters.cam_pixel_x;
    float x_angle = copter.g2.user_parameters.cam_angle_x;
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = 100.f * (x_angle * info_x / x_length / yaw_rate_tc);
    return yaw_rate_cds;
}

float ModeAttack_att::my_get_throttle_boosted(float throttle_in)
{
    float cos_tilt = copter.ahrs_view->cos_pitch() * copter.ahrs_view->cos_roll();
    float inverted_factor = 1.0f;
    float boost_factor = 1.0f / constrain_float(cos_tilt, cosf(ToRad(copter.g2.user_parameters.fly_pitch_limit*0.01f*0.67f)), 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    return throttle_out;
}