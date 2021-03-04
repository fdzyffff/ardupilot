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
    Ucam.update();
    Ugcs.update();
    Ugcs_state_update();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    Ugcs_Log_Write_UCamTarget();
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
    if (g2.user_parameters.cam_print.get() & (1<<0)) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "Raw (%0.0f,%0.0f) on:%d", Ucam.get_raw_info().x, Ucam.get_raw_info().y, Ucam.is_active());
    }
    if (g2.user_parameters.cam_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", Ucam.get_correct_info().x,Ucam.get_correct_info().y, Ucam.is_active());
    }
    if (g2.user_parameters.cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", Ucam.get_target_roll_angle()*0.01f, Ucam.get_target_pitch_rate()*0.01f, Ucam.get_target_yaw_rate()*0.01f);
    }
    if (g2.user_parameters.cam_print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "Track Angle (%0.1f)", Ucam.get_current_angle_deg());
    }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    if (ch_flag == 2) {
        copter.set_mode(Mode::Number::ATTACK, ModeReason::UNKNOWN);
    } else {
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
