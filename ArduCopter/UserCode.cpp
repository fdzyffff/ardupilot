#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM1);
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM2);
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM3);
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM4);
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM5);
    AP::user_dazhuang().add_new(AP_SerialManager::SerialProtocol_DazhuangM6);
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
    AP::user_dazhuang().read();

    if (motors->armed()) {
        float value_0 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor1)*0.5f, 0.0f, 1.0f);
        float value_1 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor2)*0.5f, 0.0f, 1.0f);
        float value_2 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor3)*0.5f, 0.0f, 1.0f);
        float value_3 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor4)*0.5f, 0.0f, 1.0f);
        float value_4 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor5)*0.5f, 0.0f, 1.0f);
        float value_5 = constrain_float(0.5f+SRV_Channels::get_output_norm(SRV_Channel::k_motor6)*0.5f, 0.0f, 1.0f);
        AP::user_dazhuang().setup(0, value_0);
        AP::user_dazhuang().setup(1, value_1);
        AP::user_dazhuang().setup(2, value_2);
        AP::user_dazhuang().setup(3, value_3);
        AP::user_dazhuang().setup(4, value_4);
        AP::user_dazhuang().setup(5, value_5);
    } else {
        AP::user_dazhuang().close();
    }
    AP::user_dazhuang().write();
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
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        if (motors->armed()) {
            AP::user_dazhuang().start();
        }
        break;
    }
    default:
        break;
    }
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
