#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    uk230.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uk230.update();
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
    if ((g2.user_parameters.cam_print.get() & (1<<0)) && uk230.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uk230.display_info.count, uk230.display_info.p1, uk230.display_info.p2, uk230.display_info.p3, uk230.display_info.p4);
        uk230.display_info.new_data = false;
        uk230.display_info.count = 0;
    }
    if (g2.user_parameters.cam_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f,%0.0f) on:%d", uk230.display_info.p11, uk230.display_info.p12, uk230.display_info.p13, uk230.is_valid());
    }
    if (g2.user_parameters.cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uk230.get_target_roll_rate(), uk230.get_target_pitch_rate(), uk230.get_target_yaw_rate());
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
