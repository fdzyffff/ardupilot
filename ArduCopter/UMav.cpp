#include "Copter.h"

UMav::UMav()
{
    _computer_ok = 0;
    _target_ok = 0;
    _mission_ok = 0;
    _all_status = 0;
}

void UMav::handle_msg(const mavlink_message_t &msg)
{
    //self check cmd 400;
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_WXBS_DO_SELFCHECK:
            handle_selfcheck(msg);
            break;
        case MAVLINK_MSG_ID_WXBS_TARGET:
            handle_target(msg);
            break;
        case MAVLINK_MSG_ID_WXBS_MISSION:
            handle_mission(msg);
            break;
        default:
            break;
    }
}

void UMav::send_status()
{
    //send status at 1Hz by 402
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                _all_status = copter.arming.pre_arm_checks(false);
                mavlink_msg_wxbs_status_send(
                    channel,
                    3,
                    _all_status);
            }
        }
    }
}

void UMav::handle_selfcheck(const mavlink_message_t &msg)
{
    //self check cmd 400;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_DO_SELFCHECK) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_DO_SELFCHECK");
        // decode packet
        mavlink_wxbs_do_selfcheck_t packet;
        mavlink_msg_wxbs_do_selfcheck_decode(&msg, &packet);
        if (packet.do_check == 1) {
            send_selfcheck();
        }
    }
}

void UMav::send_selfcheck() 
{
    //self check result 403;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_SELFCHECK_RESULT");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                uint8_t controller_ok = copter.arming.pre_arm_checks(true);
                mavlink_msg_wxbs_selfcheck_result_send(
                    channel,
                    controller_ok,
                    _computer_ok);
            }
        }
    }
}

void UMav::handle_target(const mavlink_message_t &msg)
{
    //handle target cmd 401;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_TARGET) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_TARGET");
        // decode packet
        mavlink_wxbs_target_t packet;
        mavlink_msg_wxbs_target_decode(&msg, &packet);
        _target_ok = 1;
        send_target();
    }
}

void UMav::send_target() 
{
    //send target result 404;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_TARGET_RESULT");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_target_result_send(
                    channel,
                    _target_ok);
            }
        }
    }
}

void UMav::handle_mission(const mavlink_message_t &msg)
{
    //handle mission cmd 405;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_MISSION) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_MISSION");
        // decode packet
        mavlink_wxbs_mission_t packet;
        mavlink_msg_wxbs_mission_decode(&msg, &packet);
        _mission_ok = 1;
        send_mission();
    }
}

void UMav::send_mission() 
{
    //send mission result 406;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_MISSION_RESULT");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_mission_result_send(
                    channel,
                    _mission_ok);
            }
        }
    }
}

void UMav::handle_info_test(int16_t p1)
{
    if (p1 == 402) {
        gcs().send_text(MAV_SEVERITY_INFO, "Try send 402");
        send_status();
    }
    if (p1 == 403) {
        gcs().send_text(MAV_SEVERITY_INFO, "Try send 403");
        send_selfcheck();
    }
    if (p1 == 404) {
        gcs().send_text(MAV_SEVERITY_INFO, "Try send 404");
        send_target();
    }
    if (p1 == 406) {
        gcs().send_text(MAV_SEVERITY_INFO, "Try send 406");
        send_mission();
    }
}
