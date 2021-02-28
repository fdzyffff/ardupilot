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
    return false;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack_att::run()
{
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get target lean angles
    float target_roll, target_pitch;

    my_get_target_angles(target_roll, target_pitch);

    // get target yaw rate
    float target_yaw_rate = my_get_target_yaw_rate();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(my_get_throttle_boosted(motors->get_throttle_hover()),
                                       false,
                                       g.throttle_filt);
}

void ModeAttack_att::my_get_target_angles(float &target_roll, float &target_pitch)
{
    // float pitch_target = (180.0f/M_PI) * atanf(copter.Ucam.get_raw_info().y/(0.5f*copter.g2.user_parameters.cam_pixel_y)*tanf(0.5f*copter.g2.user_parameters.cam_angle_y*(M_PI/180.f)));
    // float pitch_limit = copter.g2.user_parameters.cam_angle_y*0.35f;
    // float delta_pitch = constrain_float(pitch_target - pitch_limit, -copter.g2.user_parameters.cam_angle_y*0.5f, 5.0f)*copter.g2.user_parameters.fly_pitch_factor;
    // target_pitch = (degrees(copter.ahrs_view->pitch) + delta_pitch)*100.f;
    // float angle_limit = constrain_float(copter.aparm.angle_max, 1000.0f, attitude_control->get_althold_lean_angle_max());
    // angle_limit = MIN(angle_limit, copter.g2.user_parameters.fly_pitch_limit);
    // target_pitch = constrain_float(target_pitch,-angle_limit,angle_limit);


    float measurement = (copter.Ucam.get_raw_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y); //-1 to +0.1
    float my_target_pitch = -1.0f*copter.g2.user_parameters.Ucam_pid.update_all(0.5f, measurement, false)*copter.g2.user_parameters.fly_pitch_limit.get();
    if (my_target_pitch > 0.0f) {
        my_target_pitch *= 1.0f;
    }
    target_pitch = degrees(copter.ahrs_view->pitch)*100.f + my_target_pitch ;
    float angle_limit = constrain_float(copter.aparm.angle_max, 1000.0f, attitude_control->get_althold_lean_angle_max());
    angle_limit = MIN(angle_limit, copter.g2.user_parameters.fly_pitch_limit);
    target_pitch = constrain_float(target_pitch,-angle_limit,angle_limit);
    target_roll = 0.0f;
    //my_get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void ModeAttack_att::my_get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const
{
    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

    // roll_out and pitch_out are returned
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