#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    ubim.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    ubim.update();
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
    user_set_origin();
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

void Copter::user_set_origin()
{
    static bool ekf_set = false;
    if (g2.user_parameters.set_origin.get() == 0) {
        return;
    }

    if (ekf_set) {
        return;
    }

    AP_AHRS &user_ahrs = AP::ahrs();

    // check if EKF origin has already been set
    Location ekf_origin;
    if (user_ahrs.get_origin(ekf_origin)) {
        // gcs().send_text(MAV_SEVERITY_INFO, "Warning, current ekf origin changed!");
        return;
    }

    // if (!ekf_has_absolute_position()) {
    //     // gcs().send_text(MAV_SEVERITY_INFO, "Warning, current ekf origin changed!");
    //     return;
    // }

    Location loc;
    loc.lat = 399788219;
    loc.lng = 1163397914;
    loc.alt = 5300;
    loc.set_alt_cm(loc.alt, Location::AltFrame::ABSOLUTE);

    if (!user_ahrs.set_origin(loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Fail, user set ekf origin!");
        return;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "user set ekf origin!");
        gcs().send_text(MAV_SEVERITY_INFO, "%f, %f, %.1f", 39.9788219, 116.3397914, 53.00);
        ekf_set = true;
    }
}
