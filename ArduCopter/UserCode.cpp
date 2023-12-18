#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    float test = 1.0;
    test += 1.0;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::user_update_bi_angle() 
{
    // run custom controller after here
     Quaternion attitude_body, attitude_target;
    ahrs_view->get_quat_body_to_ned(attitude_body);

    attitude_target = attitude_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    attitude_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);
    float pitch_range = constrain_float(g2.user_parameters.user_pitch_range.get(), 1500.f, 5500.f);
    float pitch_out = degrees(attitude_error.y)*100.f/pitch_range;
    pitch_out = constrain_float(pitch_out, -1.0f, 1.0f);
    attitude_control->actuator_pitch_sysid(pitch_out);
}