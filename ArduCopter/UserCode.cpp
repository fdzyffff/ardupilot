#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    ua8.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    ua8.update();
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
    ua8.do_print();
    ua8.test();
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

void Copter::user_update_assit(float &target_roll, float &target_pitch)
{
    if (is_zero(g2.user_parameters.assit_gain.get())) {return;}
    if (!position_ok() || !motors->armed()) {
        return;
    }
    float kp = g2.user_parameters.assit_gain.get();
    Vector3f vec;
    if (!ahrs.get_velocity_NED(vec)) {
        return;
    }
    Vector2f bf_vel = ahrs.earth_to_body2D(vec.xy());
    float assit_roll = -bf_vel.y*100.f*kp;
    float assit_pitch = bf_vel.x*100.f*kp;
    float assit_max = 15.f*100.f;
    if (target_roll >= 0.0f && assit_roll > 0.0f) {
        target_roll = constrain_float(target_roll, assit_roll, assit_max);
    }
    if (target_roll <= 0.0f && assit_roll < 0.0f) {
        target_roll = constrain_float(target_roll, -assit_max, assit_roll);
    }
    if (target_pitch >= 0.0f && assit_pitch > 0.0f) {
        target_pitch = constrain_float(target_pitch, assit_pitch, assit_max);
    }
    if (target_pitch <= 0.0f && assit_pitch < 0.0f) {
        target_pitch = constrain_float(target_pitch, -assit_max, assit_pitch);
    }
}

