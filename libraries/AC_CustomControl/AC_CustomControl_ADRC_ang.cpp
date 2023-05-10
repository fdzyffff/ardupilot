#include "AC_CustomControl_ADRC_ang.h"

#if CUSTOMCONTROL_ADRC_ANG_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC_ang::var_info[] = {

    AP_SUBGROUPINFO(_adrc_atti_rate_roll, "ANG_RLL_", 1, AC_CustomControl_ADRC_ang, AC_ADRC),
    AP_SUBGROUPINFO(_adrc_atti_rate_pitch, "ANG_PIT_", 2, AC_CustomControl_ADRC_ang, AC_ADRC),
    AP_SUBGROUPINFO(_adrc_atti_rate_yaw, "ANG_YAW_", 3, AC_CustomControl_ADRC_ang, AC_ADRC),

    AP_SUBGROUPINFO(_pid_atti_rate_roll, "RAT_RLL_", 4, AC_CustomControl_ADRC_ang, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_pitch, "RAT_PIT_", 5, AC_CustomControl_ADRC_ang, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_yaw, "RAT_YAW_", 6, AC_CustomControl_ADRC_ang, AC_PID),

    AP_GROUPEND
};

AC_CustomControl_ADRC_ang::AC_CustomControl_ADRC_ang(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _adrc_atti_rate_roll(AC_ATC_MULTI_RATE_RP_B0, AC_ATC_MULTI_RATE_RP_KP, AC_ATC_MULTI_RATE_RP_KD, AC_ATC_MULTI_RATE_RP_KA, AC_ATC_MULTI_RATE_RP_KB, AC_ATC_MULTI_RATE_RP_KC, AC_ATC_MULTI_RATE_RP_OMEGA0, AC_ATC_MULTI_RATE_RP_IMAX, 0.0f, 0.0f, 0.0f),
    _adrc_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_B0, AC_ATC_MULTI_RATE_RP_KP, AC_ATC_MULTI_RATE_RP_KD, AC_ATC_MULTI_RATE_RP_KA, AC_ATC_MULTI_RATE_RP_KB, AC_ATC_MULTI_RATE_RP_KC, AC_ATC_MULTI_RATE_RP_OMEGA0, AC_ATC_MULTI_RATE_RP_IMAX, 0.0f, 0.0f, 0.0f),
    _adrc_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_B0, AC_ATC_MULTI_RATE_YAW_KP, AC_ATC_MULTI_RATE_YAW_KD, AC_ATC_MULTI_RATE_YAW_KA, AC_ATC_MULTI_RATE_YAW_KB, AC_ATC_MULTI_RATE_YAW_KC, AC_ATC_MULTI_RATE_YAW_OMEGA0, AC_ATC_MULTI_RATE_YAW_IMAX, 0.0f, 0.0f, 0.0f),
    _pid_atti_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AC_CustomControl_ADRC_ang::update()
{
      // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // run custom controller after here
     Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // // recalculate ang vel feedforward from attitude target model
    // // rotation from the target frame to the body frame
    // Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    // // target angle velocity vector in the body frame
    // Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    // Vector3f attitude_target_euler;
    // attitude_target.to_euler(attitude_target_euler.x, attitude_target_euler.y, attitude_target_euler.z);
    // Vector3f attitude_body_euler;
    // attitude_body.to_euler(attitude_body_euler.x, attitude_body_euler.y, attitude_body_euler.z);

    // run attitude controller
    Vector3f target_rate;
    // target_rate[0] = _p_angle_roll2.kP() * attitude_error.x + ang_vel_body_feedforward[0];
    // target_rate[1] = _p_angle_pitch2.kP() * attitude_error.y + ang_vel_body_feedforward[1];
    // target_rate[2] = _p_angle_yaw2.kP() * attitude_error.z + ang_vel_body_feedforward[2];

    target_rate[0] = _adrc_atti_rate_roll.update_all(0.0f, -attitude_error[0], _dt);
    target_rate[1] = _adrc_atti_rate_pitch.update_all(0.0f, -attitude_error[1], _dt);
    target_rate[2] = _adrc_atti_rate_yaw.update_all(0.0f, -attitude_error[2], _dt);
    // run rate controller
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = _pid_atti_rate_roll.update_all(target_rate[0], gyro_latest[0], _dt);
    motor_out.y = _pid_atti_rate_pitch.update_all(target_rate[1], gyro_latest[1], _dt);
    motor_out.z = _pid_atti_rate_yaw.update_all(target_rate[2], gyro_latest[2], _dt);

    return motor_out;
}

// This example uses exact same controller architecture as ArduCopter attitude controller without all the safe guard against saturation.
// The gains are scaled 0.9 times to better detect switch over response. 
// Note that integrator are not reset correctly as it is done in reset_main_att_controller inside AC_CustomControl.cpp
// This is done intentionally to demonstrate switch over performance of two exact controller with different reset handling.
void AC_CustomControl_ADRC_ang::reset(void)
{
    _adrc_atti_rate_roll.reset_I();
    _adrc_atti_rate_pitch.reset_I();
    _adrc_atti_rate_yaw.reset_I();
    _adrc_atti_rate_roll.reset_filter();
    _adrc_atti_rate_pitch.reset_filter();
    _adrc_atti_rate_yaw.reset_filter();
}

#endif
