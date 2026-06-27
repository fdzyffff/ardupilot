#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    uart.init();
    uattack.init();
    udelay.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uattack.update();
    uart.update();
    udelay.push();
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
    uattack.do_print();
    uart.do_print();
    update_ls_status();
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

void Copter::user_handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_LS_CMD) {
        mavlink_ls_cmd_t packet;
        mavlink_msg_ls_cmd_decode(&msg, &packet);
        bool use_alt = packet.flag & (1<<0);
        bool use_latlng = packet.flag & (1<<1);
        bool use_yaw = packet.flag & (1<<2);
        // bool use_radius = packet.flag & (1<<3);
        switch(packet.type) {
        case 1:
            {
                if (copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
                    Location target_loc{copter.current_loc};
                    if (!copter.ap.land_complete) {
                        target_loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()), ahrs.groundspeed_vector().length());
                    }
                    float target_yaw_cd = 0.0f;
                    if (use_yaw) {
                        target_yaw_cd = packet.yaw_cd;
                    } else {
                        target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                    }

                    if (use_alt && copter.ap.land_complete) {
                        target_loc.alt = packet.alt;
                    }
                    if (use_latlng && copter.ap.land_complete) {
                        target_loc.lat = packet.lat;
                        target_loc.lng = packet.lng;
                    }
                    mode_guided.set_destination(target_loc, true, target_yaw_cd, false, 0.0f, false);
                }
                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "ls cmd t1");}
            }
            break;
        case 2:
            {
                if (copter.set_mode(Mode::Number::CIRCLE, ModeReason::GCS_COMMAND)) {
                    Location target_loc{copter.current_loc};
                    target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
                    if (use_alt) {
                        target_loc.alt = packet.alt;
                    }
                    if (use_latlng) {
                        target_loc.lat = packet.lat;
                        target_loc.lng = packet.lng;
                    }
                    copter.circle_nav->set_center(target_loc);
                }
                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "ls cmd t2");}
            }
            break;
        case 3:
            {
                if (copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
                    float target_yaw_cd = 0.0f;
                    if (use_yaw) {
                        target_yaw_cd = packet.yaw_cd;
                    } else {
                        target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                    }

                    float target_alt_m = 1.0f;
                    if (use_alt) {
                        target_alt_m = (float)packet.alt * 0.01f;
                    }

                    if (mode_guided.do_user_takeoff_start(target_alt_m * 100.0f)) {
                        copter.set_auto_armed(true);
                        mode_guided.auto_yaw.set_fixed_yaw(target_yaw_cd*0.01f, 0.0f, 0, false);
                    }


                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "target_alt_cm %f ", target_alt_m * 100.f);}
                }

                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "ls cmd t3");}
            }
            break;
        case 4:
            {
                if (copter.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
                    Location target_loc{AP::ahrs().get_home()};
                    if (use_latlng) {
                        target_loc.lat = packet.lat;
                        target_loc.lng = packet.lng;
                    }
                    copter.mode_rtl.set_return_loc(target_loc);
                }
                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "ls cmd t4");}
            }
            break;
        case 5:
            {
                if (copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
                    ;
                }

                if (uart.print.get() & (1<<4)) {gcs().send_text(MAV_SEVERITY_INFO, "ls cmd t5");}
            }
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Unknow ls cmd [%d]", packet.type);
        }
    }
}


void Copter::update_ls_status()
{
    gcs().send_message(MSG_LS_STATUS);
}

void Copter::send_ls_status(mavlink_channel_t chan)
{
    uint8_t status = 0;// 1: mc mode, 2: fw mode, 3: takeoff, 4: return, 5: land, 6: external control
    if (flightmode->mode_number() == Mode::Number::GUIDED) {
        status = 1;
        if (flightmode->is_taking_off()) {
            status = 3;
        }
    }
    if (flightmode->mode_number() == Mode::Number::CIRCLE || (flightmode->mode_number() == Mode::Number::EXTERNAL && !mode_external.is_attack())) {
        status = 2;
    }
    if (flightmode->mode_number() == Mode::Number::RTL) {
        status = 4;
    }
    if (flightmode->mode_number() == Mode::Number::LAND) {
        status = 5;
    }
    if (flightmode->mode_number() == Mode::Number::EXTERNAL && mode_external.is_attack()) {
        status = 6;
    }
    uint8_t collision_triggered = 0;
    mavlink_msg_ls_status_send(
        chan,
        status,
        collision_triggered);
}
