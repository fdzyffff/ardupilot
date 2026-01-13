#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"
#include <FD_CAN/FD_CAN.h>
#include <AP_CANManager/AP_CANManager.h>

extern const AP_HAL::HAL& hal;

void FD_DATA::send_hxts_hy_bms_c1(mavlink_channel_t chan)
{
    mavlink_msg_hxts_hy_bms_c1_send_struct(chan, &hxts_hy_bms_c1_packet);
}

void FD_DATA::send_hxts_hy_bms_c2(mavlink_channel_t chan)
{
    mavlink_msg_hxts_hy_bms_c2_send_struct(chan, &hxts_hy_bms_c2_packet);
}

void FD_DATA::send_hxts_hy_bms_c3(mavlink_channel_t chan)
{
    mavlink_msg_hxts_hy_bms_c3_send_struct(chan, &hxts_hy_bms_c3_packet);
}

void FD_DATA::do_switch(bool switch_on) {
    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        if (AP::can().get_driver_type(i) == AP_CAN::Protocol::FDCAN) {
            FD_CAN *fd_can = FD_CAN::get_can_fd(i);
            if (fd_can == nullptr) {
                gcs().send_text(MAV_SEVERITY_INFO, "%d| fd_can == nullptr", i);
                continue;
            }
            if (fd_can->_bms_ptr == nullptr) {
                gcs().send_text(MAV_SEVERITY_INFO, "%d| fd_can->_bms_ptr == nullptr", i);
                continue;
            }
            fd_can->_bms_ptr->set_switch(switch_on);
            gcs().send_text(MAV_SEVERITY_INFO, "%d| fd_can->_bms_ptr switch %d", i, switch_on);
            break;
        }
    }
}

// void FD_DATA::handle_message_power_control(const mavlink_message_t &msg)
// {
//     uint8_t final_cmd = 233;

//     if (msg.msgid == MAVLINK_MSG_ID_POWER_CONTROL) {
//         mavlink_power_control_t packet;
//         mavlink_msg_power_control_decode(&msg, &packet);

//         if (packet.target_system == 0 || packet.target_system == gcs().sysid_this_mav()) {
//             if (packet.command == 1) {
//                 final_cmd = 1;
//             }
//             if (packet.command == 2) {
//                 final_cmd = 0;
//             }
//         }
//     }

//     if (final_cmd == 233) {
//         return;
//     }

//     for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
//         if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_FDCAN) {
//             FD_CAN *fd_can = FD_CAN::get_can_fd(i);
//             if (fd_can == nullptr) {
//                 // send_text(MAV_SEVERITY_INFO, "%d| fd_can == nullptr", i);
//                 continue;
//             }
//             if (fd_can->_bms_ptr == nullptr) {
//                 if (fd_can->_print.get()) {
//                     gcs().send_text(MAV_SEVERITY_INFO, "%d| fd_can->_bms_ptr", i);
//                     continue;
//                 }
//             }
//             fd_can->_bms_ptr->set_switch(final_cmd);
//             break;
//         }
//     }
// }
