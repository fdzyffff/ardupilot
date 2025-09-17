#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    uart.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uart.update();
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