#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    uart.init();
    uattack.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uart.update();
    uattack.update();
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
    gcs().send_message(MSG_HXTS_BAT_CAN_STATUS);
    AP::fd_data().set_is_flying(copter.motors->armed() && (!ap.land_complete));
    AP::fd_data().update();

    // put your 1Hz code here
    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.count_log, uattack.display_info.p1, uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d", uattack.get_ef_info().x,uattack.get_ef_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    // if (uattack.print.get() & (1<<3)) { // 8
    //     gcs().send_text(MAV_SEVERITY_WARNING, "ar (%0.1f , %0.1f , %0.2f , %0.2f)", uattack._attack_angle_target, uattack._attack_angle_measure, uattack._attack_angle_rate_target, uattack._attack_angle_rate_measure);
    // }
    // if (uattack.print.get() & (1<<4)) { // 16
    //     gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f , %0.1f , %0.1f , %0.2f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate(), uattack.attack_throttle.get());
    // }
    // if (uattack.print.get() & (1<<5)) { // 32
    //     gcs().send_text(MAV_SEVERITY_WARNING, "apid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack._attack_throttle_pid, uattack._attack_throttle_p, uattack._attack_throttle_i, uattack._attack_throttle_d);
    // }
    if (uattack.print.get() & (1<<6)) { // 364
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.p11, uattack.display_info.p12, uattack.display_info.p13, uattack.display_info.p14);
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
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        // AP::fd_data().do_switch(0);
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        // AP::fd_data().do_switch(1);
        break;
    }
}
#endif

// void Copter::handle_message_for_mission_test(const mavlink_message_t &msg)
// {
//     if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
//         mavlink_command_long_t packet;
//         mavlink_msg_command_long_decode(&msg, &packet);
//         switch(packet.command) {
//             case MAV_CMD_USER_4:
//                 {
//                     _last_ms = millis();
//                     float theta1 =  cal_frame_angle(cam_width.get(), cam_angle_x.get(), packet.param1); // x-axis, degree
//                     float theta2 =  cal_frame_angle(cam_height.get(), cam_angle_y.get(), packet.param2); // y-axis, degree

//                     Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
//                     float p1 =  degrees(atanf(tmp.y/tmp.x));
//                     float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
//                     handle_info(p1, p2);
//                     // handle_info(theta1, theta2);
//                 }
//                 break;
//             default:
//                 break;
//         }
//     }
// }