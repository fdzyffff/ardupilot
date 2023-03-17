#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    FD1_uart_init();
    netgun.Init();

    user_stat.nacelle_byte_count = 0;
    user_stat.gcs_byte_count = 0;
    user_stat.nacelle_valid_byte_count = 0;
    user_stat.gcs_valid_byte_count = 0;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    FD1_uart_update();
    netgun.Update();
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
    if (g2.user_parameters.stat_print.get() & (1<<0)) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "up %d/s[%d], down %d/s[%d]", user_stat.gcs_byte_count, user_stat.gcs_valid_byte_count, user_stat.nacelle_byte_count, user_stat.nacelle_valid_byte_count);
    }
    user_stat.nacelle_byte_count = 0;
    user_stat.gcs_byte_count = 0;
    user_stat.nacelle_valid_byte_count = 0;
    user_stat.gcs_valid_byte_count = 0;
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
