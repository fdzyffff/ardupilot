#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    AP::ef_counter().update();
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
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    if (ch_flag == 2) {
        AP::ef_counter().EFGate_reset();
        gcs().send_text(MAV_SEVERITY_INFO, "Timer reset");
    }
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    if (ch_flag == 2) {
        Location temp_loc;
        if (ahrs.get_location(temp_loc)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Lat %d.%ld", temp_loc.lat/10000000, labs(temp_loc.lat%10000000));
            gcs().send_text(MAV_SEVERITY_INFO, "Lng %d.%ld", temp_loc.lng/10000000, labs(temp_loc.lng%10000000));
            gcs().send_text(MAV_SEVERITY_INFO, "Lat %d", temp_loc.lat);
            gcs().send_text(MAV_SEVERITY_INFO, "Lng %d", temp_loc.lng);
        }
    }
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
