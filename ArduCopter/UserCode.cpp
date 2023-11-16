#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    useruartfwd.init();
    SRV_Channels::set_range(SRV_Channel::k_yaw_out_left, 100);
    SRV_Channels::set_range(SRV_Channel::k_yaw_out_right, 100);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    float temp_yaw_out_left  = 0.0f;
    float temp_yaw_out_right = 0.0f;
    if (motors->armed()) {
        float temp_yaw = motors->get_yaw() + motors->get_yaw_ff();
        temp_yaw_out_left  = constrain_float(-temp_yaw, 0.0f, 1.0f)*100.f;
        temp_yaw_out_right = constrain_float( temp_yaw, 0.0f, 1.0f)*100.f;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out_left, temp_yaw_out_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out_right, temp_yaw_out_right);
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    useruartfwd.update();
    float temp_yaw_out_left = -100.0f;
    float temp_yaw_out_right = -100.0f;
    if (motors->armed()) {
        float temp_yaw_out = motors->get_yaw() + motors->get_yaw_ff();
        temp_yaw_out_left  = constrain_float(-temp_yaw_out*0.5f + 0.5f, 0.4f, 1.f)*100.f;
        temp_yaw_out_right = constrain_float( temp_yaw_out*0.5f + 0.5f, 0.4f, 1.f)*100.f;
    }
        // temp_yaw_out_left  = constrain_float(.0f, -1.0f, 1.0f)*100.f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out_left, temp_yaw_out_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out_right, temp_yaw_out_right);
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
