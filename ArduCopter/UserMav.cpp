#include "Copter.h"


void UserMAV_handle_selfcheck(const mavlink_message_t &msg)
{
    // //self check cmd 400;
    // if (msg.msgid == MAVLINK_MSG_ID_WXBS_DO_SELFCHECK) {
    //     // decode packet
    //     mavlink_bxbs_do_selfcheck_t packet;
    //     mavlink_msg_command_int_decode(&msg, &packet);
    //     switch (packet.command) {
    //         case MAV_CMD_USER_1:
    //             _target_vel = packet.param1; // m/s
    //             _target_bearing = packet.param2; // degree
    //             _raw_target_loc.lat = packet.x; // 1e7 degree
    //             _raw_target_loc.lng = packet.y; // 1e7 degree
    //             _raw_target_loc.alt = (int32_t)packet.z; // cm
    //             _last_update_ms = millis();
    //             break;
    //         default:
    //             break;
    //     }
    // }
}

void UserMAV_send_selfcheck() 
{
    //self check result 403;
}

void UserMAV_send_status()
{
    //send status at 1Hz by 402
}

void UserMAV_handle_target(const mavlink_message_t &msg)
{
    //handle target cmd 401;
}

void UserMAV_send_target() 
{
    //send target result 404;
}

void UserMAV_handle_mission(const mavlink_message_t &msg)
{
    //handle mission cmd 405;
}

void UserMAV_send_mission() 
{
    //send mission result 406;
}