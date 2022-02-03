#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    if (g2.user_parameters.EF_use_uwb_port.get() > 0) {
        AP::ef_counter().EFGate_reset();
        gcs().send_text(MAV_SEVERITY_INFO, "Timer reset");
    }

    if (g2.user_parameters.EF_football_dir.get() >= 0.0f) {
        gcs().send_text(MAV_SEVERITY_INFO, "Head set: %0.2f", g2.user_parameters.EF_football_dir.get());
    }
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    AP::ef_counter().update();

    if (copter.arming.is_armed() && (flightmode == &mode_attef2 || flightmode == &mode_attef3 || flightmode == &mode_posef2 || flightmode == &mode_posef3)) {
        float height = copter.inertial_nav.get_altitude() * 0.01 - copter.arming_altitude_m;
        if (height > 600.f && !rangefinder_alt_ok()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RNGFND Warning! LAND");
            set_mode(Mode::Number::LAND, ModeReason::UNAVAILABLE);
        }
    }

    if (flightmode == &mode_land && copter.arming.is_armed()) {
        if (copter.rangefinder_alt_ok()) {
            if (copter.rangefinder_state.alt_cm < 20.f) {
                copter.arming.disarm();
                gcs().send_text(MAV_SEVERITY_WARNING, "DISARM 1");
            }
        } else {
            copter.arming.disarm();
            gcs().send_text(MAV_SEVERITY_WARNING, "DISARM 2");
        }
    }
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
    if (g2.user_parameters.EF_use_uwb_port.get() <= 0) {return;}
    // put your 1Hz code here
    AP_HAL::UARTDriver *_port;
    if (!(_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, g2.user_parameters.EF_use_uwb_port.get()-1))) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No UWB port");
        return;
    }
    AP::ef_counter().uart_send(_port, g.sysid_this_mav.get());
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
    switch (ch_flag) {
    case 2:
        copter.arming.arm(AP_Arming::Method::AUXSWITCH);
        // remember that we are using an arming switch, for use by set_throttle_zero_flag
        copter.ap.armed_with_switch = true;
        break;
    case 1:
        // nothing
        break;
    case 0:
        if (!copter.arming.is_armed()) {
            return;
        } else if (copter.rangefinder_alt_ok()) {
            set_mode(Mode::Number::LAND, ModeReason::UNAVAILABLE);
        } else {
            copter.arming.disarm();
        }
        break;
    }
}
#endif
