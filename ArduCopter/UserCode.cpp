#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    ufence.init();
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
    ufence.update();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    user_ekf_switch();
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
    // gcs().send_message(MSG_ZF8888_STATUS); //电子桩, F4
    gcs().send_message(MSG_ZF6666_STATUS); //飞控, H7
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

bool Copter::user_arm_switch_count() {
    static uint32_t last_ms = millis();
    static uint8_t last_count = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_ms < 1500) {
        last_ms = now_ms;
        last_count++;
    } else {
        last_ms = now_ms;
        last_count = 0;
        last_count++;
    }
    if (last_count >= 3) {
        last_count = 0;
        return true;
    }
    return false;
}

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

void Copter::user_ekf_switch()
{
    
}