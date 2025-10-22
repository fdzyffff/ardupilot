#include "Copter.h"

void UMav::handle_mission_msg(const mavlink_message_t &msg)
{
    trans_status.handle_mission_msg(msg);
    trans_selfcheck.handle_mission_msg(msg);
    trans_target.handle_mission_msg(msg);
    trans_mission.handle_mission_msg(msg);
    trans_relay_positon.handle_mission_msg(msg);

    if (msg.msgid == MAVLINK_MSG_ID_WXBS_ATTACK_CMD) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_ATTACK_CMD");
        // decode packet
        mavlink_wxbs_attack_cmd_t packet;
        mavlink_msg_wxbs_attack_cmd_decode(&msg, &packet);
        if (packet.attack_cmd == 1) {
            copter.set_mode(Mode::Number::ATTACK, ModeReason::GCS_COMMAND);
            if (!copter.motors->armed()) {
                // if disarmed, arm motors
                copter.arming.arm(AP_Arming::Method::MAVLINK);
                gcs().send_text(MAV_SEVERITY_INFO, "Set ATTACK Mode");
            }
        }
    }

    if (msg.msgid == MAVLINK_MSG_ID_WXBS_NAV_CMD) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_NAV_CMD");
        // decode packet
        mavlink_wxbs_nav_cmd_t packet;
        mavlink_msg_wxbs_nav_cmd_decode(&msg, &packet);
        if (packet.ekf_source <= 2) {
            AP::ahrs().set_posvelyaw_source_set(packet.ekf_source);
            gcs().send_text(MAV_SEVERITY_INFO, "WXBS_NAV_CMD : %d", packet.ekf_source);
        }
    }

    // //self check cmd 400;
    // switch (msg.msgid) {
    //     case MAVLINK_MSG_ID_WXBS_DO_SELFCHECK:
    //         handle_selfcheck(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_TARGET:
    //         handle_target(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_TARGET_RESULT:
    //         handle_target_result(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_SELFCHECK_RESULT:
    //         handle_selfcheck_result(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_STATUS:
    //         handle_status(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_RELAY_POSITION:
    //         handle_relay_position(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_RELAY_POSITION_RESULT:
    //         handle_relay_position_result(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_MISSION:
    //         handle_mission(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_MISSION_RESULT:
    //         handle_mission_result(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_ATTACK_INFO:
    //         handle_attack_info(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_ATTACK_CMD:
    //         handle_attack_cmd(msg);
    //         break;
    //     case MAVLINK_MSG_ID_WXBS_NAV_CMD:
    //         handle_nav_cmd(msg);
    //         break;
    //     default:
    //         break;
    // }
}

// void UMav::handle_selfcheck(const mavlink_message_t &msg)
// {
//     //self check cmd 400;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_DO_SELFCHECK) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_DO_SELFCHECK");
//         // decode packet
//         mavlink_wxbs_do_selfcheck_t packet;
//         mavlink_msg_wxbs_do_selfcheck_decode(&msg, &packet);
//         if (packet.do_check == 1) {
//             send_selfcheck_result();
//         }
//     }
// }

// void UMav::handle_target(const mavlink_message_t &msg)
// {
//     //handle target cmd 401;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_TARGET) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_TARGET");
//         // decode packet
//         mavlink_wxbs_target_t packet;
//         mavlink_msg_wxbs_target_decode(&msg, &packet);
//         _target_ok = 1;
//         // send_target();
//         send_target_result();
//     }
// }

// void UMav::handle_target_result(const mavlink_message_t &msg)
// {
//     //handle target cmd 401;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_TARGET_RESULT) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_TARGET_RESULT");
//         // decode packet
//     }
// }

// void UMav::handle_selfcheck_result(const mavlink_message_t &msg)
// {
//     //handle target cmd 401;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_SELFCHECK_RESULT) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_SELFCHECK_RESULT");
//         // decode packet
//     }
// }


// void UMav::handle_status(const mavlink_message_t &msg)
// {
//     //handle target cmd 401;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_STATUS) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_STATUS");
//         // decode packet
//     }
// }


// void UMav::handle_mission(const mavlink_message_t &msg)
// {
//     //handle mission cmd 405;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_MISSION) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_MISSION");
//         // decode packet
//         mavlink_wxbs_mission_t packet;
//         mavlink_msg_wxbs_mission_decode(&msg, &packet);
//         _mission_ok = 1;
//         send_mission_result();
//     }
// }

// void UMav::handle_mission_result(const mavlink_message_t &msg)
// {
//     //handle mission cmd 405;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_MISSION_RESULT) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_MISSION_RESULT");
//         // decode packet
//     }
// }

// void UMav::handle_relay_position(const mavlink_message_t &msg)
// {
//     //handle relay cmd 407;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_RELAY_POSITION) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_RELAY_POSITION");
//         // decode packet
//         mavlink_wxbs_relay_position_t packet;
//         mavlink_msg_wxbs_relay_position_decode(&msg, &packet);
//         send_relay_position_result();
//     }
// }

// void UMav::handle_relay_position_result(const mavlink_message_t &msg)
// {
//     //handle relay cmd 407;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_RELAY_POSITION_RESULT) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_RELAY_POSITION_RESULT");
//         // decode packet
//     }
// }


// void UMav::handle_attack_info(const mavlink_message_t &msg)
// {
//     //handle attack info 410;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_ATTACK_INFO) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_ATTACK_INFO");
//         // decode packet
//         mavlink_wxbs_attack_info_t packet;
//         mavlink_msg_wxbs_attack_info_decode(&msg, &packet);
//     }
// }

// void UMav::handle_attack_cmd(const mavlink_message_t &msg)
// {
//     //handle attack cmd 411;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_ATTACK_CMD) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_ATTACK_CMD");
//         // decode packet
//         mavlink_wxbs_attack_cmd_t packet;
//         mavlink_msg_wxbs_attack_cmd_decode(&msg, &packet);
//     }
// }

// void UMav::handle_nav_cmd(const mavlink_message_t &msg)
// {
//     //handle nav cmd 412;
//     if (msg.msgid == MAVLINK_MSG_ID_WXBS_NAV_CMD) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_NAV_CMD");
//         // decode packet
//         mavlink_wxbs_nav_cmd_t packet;
//         mavlink_msg_wxbs_nav_cmd_decode(&msg, &packet);
//     }
// }

// void UMav::send_do_selfcheck() 
// {
//     //self check result 400;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_DO_SELFCHECK");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 uint8_t controller_ok = copter.arming.pre_arm_checks(true);
//                 mavlink_msg_wxbs_do_selfcheck_send(
//                     channel,
//                     controller_ok);
//             }
//         }
//     }
// }

// void UMav::send_target() 
// {
//     //send target result 401;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_TARGET");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 mavlink_msg_wxbs_target_send(
//                     channel,
//                     123,
//                     234,
//                     345,
//                     567,
//                     678,
//                     789);
//             }
//         }
//     }
// }

// void UMav::send_status()
// {
//     //send status at 1Hz by 402
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 _all_status = copter.arming.pre_arm_checks(false);
//                 mavlink_msg_wxbs_status_send(
//                     channel,
//                     3,
//                     _all_status,
//                     0);
//             }
//         }
//     }
// }

// void UMav::send_selfcheck_result() 
// {
//     //self check result 403;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_SELFCHECK_RESULT");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 uint8_t controller_ok = copter.arming.pre_arm_checks(true);
//                 mavlink_msg_wxbs_selfcheck_result_send(
//                     channel,
//                     controller_ok,
//                     _computer_ok);
//             }
//         }
//     }
// }

// void UMav::send_target_result() 
// {
//     //send target result 404;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_TARGET_RESULT");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 mavlink_msg_wxbs_target_result_send(
//                     channel,
//                     _target_ok);
//             }
//         }
//     }
// }

// void UMav::send_mission() 
// {
//     //send mission 405;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_MISSION");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 int32_t lat[10];
//                 int32_t lng[10];
//                 int32_t alt[10];
//                 for (uint8_t i_pos = 0; i_pos < 10; i_pos++) {
//                     lat[i_pos] = 1;
//                     lng[i_pos] = 1;
//                     alt[i_pos] = 1;
//                 }
//                 mavlink_msg_wxbs_mission_send(
//                     channel,
//                     1,
//                     2,
//                     3,
//                     10,
//                     lat,
//                     lng,
//                     alt);
//             }
//         }
//     }
// }

// void UMav::send_mission_result() 
// {
//     //send mission result 406;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_MISSION_RESULT");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 mavlink_msg_wxbs_mission_result_send(
//                     channel,
//                     _mission_ok);
//             }
//         }
//     }
// }

// void UMav::send_relay_position() 
// {
//     //send mission result 407;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_RELAY_POSITION");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 int32_t lat[5];
//                 int32_t lng[5];
//                 int32_t alt[5];
//                 for (uint8_t i_pos = 0; i_pos < 5; i_pos++) {
//                     lat[i_pos] = 1;
//                     lng[i_pos] = 1;
//                     alt[i_pos] = 1;
//                 }
//                 mavlink_msg_wxbs_relay_position_send(
//                     channel,
//                     1,
//                     2,
//                     3,
//                     lat,
//                     lng,
//                     alt);
//             }
//         }
//     }
// }


// void UMav::send_relay_position_result() 
// {
//     //send relay result 408;
//     gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_RELAY_POSITION_RESULT");
//     uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
//     for (uint8_t i=0; i<gcs().num_gcs(); i++) {
//         mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
//         if (mask & (1U<<i)) {
//             if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
//                 mavlink_msg_wxbs_relay_position_result_send(
//                     channel,
//                     1);
//             }
//         }
//     }
// }

void UMav::send_apm_status()
{
    //send apm status 413;
    // gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_APM_STATUS");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_apm_status_send(
                    channel,
                    AP::ahrs().get_posvelyaw_source_set(),
                    0,
                    copter.upayload.get_status());
            }
        }
    }
}
