#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    uengines.Init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uengines.Update();
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
    if (ch_flag == RC_Channel::AuxSwitchPos::HIGH || ch_flag == RC_Channel::AuxSwitchPos::MIDDLE) 
    {
        bool pass_check = true;
        if (pass_check) {
            gcs().send_text(MAV_SEVERITY_INFO, "Do start");
            uengines.set_state(UserEngines::UserEnginesState::Start);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Err: Start");
        }
    }
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
    if (ch_flag == RC_Channel::AuxSwitchPos::HIGH || ch_flag == RC_Channel::AuxSwitchPos::MIDDLE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Do stop");
        uengines.set_state(UserEngines::UserEnginesState::Stop);
    }
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
