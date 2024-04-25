#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    useruartfwd.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    useruartfwd.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
void Copter::userhook_25Hz()
{
    // put your 25Hz code here
    ugimbal.update();
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
    if (ugimbal.display_info.count > 0) {
        if ((g2.user_parameters.cam_print.get() & (1<<0)) ) { // 1
            gcs().send_text(MAV_SEVERITY_WARNING, "1[%d] %0.0f,%0.0f,%0.0f,%0.0f", ugimbal.display_info.count, ugimbal.display_info.p1, ugimbal.display_info.p2, ugimbal.display_info.p3, ugimbal.display_info.p4);
        }
        if ((g2.user_parameters.cam_print.get() & (1<<1)) ) { // 2
            gcs().send_text(MAV_SEVERITY_WARNING, "2[%d] %0.0f,%0.0f,%0.0f,%0.0f", ugimbal.display_info.count, ugimbal.display_info.p11, ugimbal.display_info.p12, ugimbal.display_info.p13, ugimbal.display_info.p14);
        }
        ugimbal.display_info.count = 0;
    }
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
