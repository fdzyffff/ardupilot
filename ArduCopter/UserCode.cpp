#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    uart.init();
    uattack.init();
    yolo_drop.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    uart.update();
    uattack.update();
    yolo_drop.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    gcs().send_message(MSG_HXTS_BAT_CAN_STATUS);
    AP::fd_data().set_is_flying(copter.motors->armed() && (!ap.land_complete));
    AP::fd_data().update();

    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) {
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f",
            uattack.display_info.count_log, uattack.display_info.p1,
            uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d",
            uattack.get_ef_info().x, uattack.get_ef_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d",
            uattack.get_ef_rate_info().x, uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<6)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f",
            uattack.display_info.p11, uattack.display_info.p12,
            uattack.display_info.p13, uattack.display_info.p14);
    }
    yolo_drop.print_debug();
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        AP::fd_data().do_switch(0);
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        AP::fd_data().do_switch(1);
        break;
    }
}
#endif
