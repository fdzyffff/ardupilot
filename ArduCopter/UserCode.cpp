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
    static bool pos_last_good = true;
    static bool pos_good = true;
    static bool need_run = false;
    static Mode *new_mode;
    static Mode *last_mode;
    if (motors->armed()) {
        pos_good = copter.position_ok();
        if (pos_last_good) {
            if (pos_good) {
                last_mode = flightmode;
                new_mode = nullptr;
            } else {
                if (last_mode->requires_GPS()) {
                    gcs().send_text(MAV_SEVERITY_INFO, "POS bad, record %s", last_mode->name());
                    need_run = true;
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "POS bad, continue");
                    need_run = false;
                }
            }
        } else if (need_run) {
            if ((new_mode == nullptr) && (last_mode != flightmode)) {
                new_mode = flightmode;
                gcs().send_text(MAV_SEVERITY_INFO, "Now in %s", new_mode->name());
            }
            if (pos_good) {
                if (new_mode == flightmode) { // 丢GPS期间模式没变化
                    gcs().send_text(MAV_SEVERITY_INFO, "POS good, recover %s", last_mode->name());
                    set_mode(last_mode->mode_number(), ModeReason::GCS_COMMAND);
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "POS good, continue %s", flightmode->name());
                }
                new_mode = nullptr;
                need_run = false;
            }
        }
        pos_last_good = pos_good;
    } else {
        last_mode = flightmode;
    }
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
