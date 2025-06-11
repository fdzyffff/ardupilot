#include "Copter.h"

// ~~~~~~~~~~~~~~~~~~ WXBS_STATUS ~~~~~~~~~~~~~~~~~~
void UMav_trans_status::update()
{
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_send_bsq_ms > repeat_time_ms) {
        send_bsq_msg();
        last_send_bsq_ms = tnow_ms;
    }
}

void UMav_trans_status::handle_mission_msg(const mavlink_message_t &msg)
{
    //handle target cmd 402;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_STATUS) {
        // gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_STATUS");
        // decode packet
        mavlink_msg_wxbs_status_decode(&msg, &packet);
        // gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_STATUS %d", packet);
    }
}

void UMav_trans_status::send_bsq_msg()
{
    //self check result 402;
    // gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_SELFCHECK_RESULT");

    mavlink_message_t msg;

    // packet.type = 0;
    packet.throw_status = 0;
    packet.battery = 3;

    UNUSED_RESULT(mavlink_msg_wxbs_status_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet));

    copter.umav.send_bsq_message(&msg);
}

// ~~~~~~~~~~~~~~~~~~ WXBS_DO_SELFCHECK ~~~~~~~~~~~~~~~~~~
void UMav_trans_selfcheck::update()
{
    if (!bsq_waiting) {return;}
    uint32_t tnow_ms = millis();
    if (tnow_ms - receive_bsq_ms > timeout_bsq_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "OUT WXBS_DO_SELFCHECK");
        out_packet.computer_ok = 0;
        send_bsq_msg();
        bsq_waiting = false;
    }

    if ((tnow_ms - last_send_mission_ms > repeat_time_ms) || (last_send_mission_ms == 0)) {
        send_mission_msg();
        last_send_mission_ms = tnow_ms;
    }
}


void UMav_trans_selfcheck::handle_bsq_msg(const mavlink_message_t &msg)
{
    //self check cmd 400;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_DO_SELFCHECK) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_DO_SELFCHECK");
        // decode packet
        mavlink_msg_wxbs_do_selfcheck_decode(&msg, &in_packet);
        if (in_packet.do_check == 1) {
            bsq_waiting = true;
            last_send_mission_ms = 0;
            receive_bsq_ms = millis();
        }
    }
}

void UMav_trans_selfcheck::send_mission_msg()
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                
                mavlink_msg_wxbs_do_selfcheck_send(
                    channel,
                    in_packet.do_check);
            }
        }
    }
}

void UMav_trans_selfcheck::handle_mission_msg(const mavlink_message_t &msg)
{
    //handle target cmd 403;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_SELFCHECK_RESULT) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_SELFCHECK_RESULT");
        // decode packet
        mavlink_msg_wxbs_selfcheck_result_decode(&msg, &out_packet);
        send_bsq_msg();
        bsq_waiting = false;
    }
}

void UMav_trans_selfcheck::send_bsq_msg()
{
    //self check result 403;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_SELFCHECK_RESULT");

    mavlink_message_t msg;

    out_packet.controller_ok = 1;

    UNUSED_RESULT(mavlink_msg_wxbs_selfcheck_result_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &out_packet));

    copter.umav.send_bsq_message(&msg);
}

// ~~~~~~~~~~~~~~~~~~ WXBS_TARGET ~~~~~~~~~~~~~~~~~~
void UMav_trans_target::update()
{
    if (!bsq_waiting) {return;}
    uint32_t tnow_ms = millis();
    if (tnow_ms - receive_bsq_ms > timeout_bsq_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "OUT WXBS_TARGET");
        out_packet.target_ok = 0;
        send_bsq_msg();
        bsq_waiting = false;
    }

    if ((tnow_ms - last_send_mission_ms > repeat_time_ms) || (last_send_mission_ms == 0)) {
        send_mission_msg();
        last_send_mission_ms = tnow_ms;
    }
}

void UMav_trans_target::handle_bsq_msg(const mavlink_message_t &msg)
{
    //handle target cmd 401;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_TARGET) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_TARGET");
        // decode packet
        mavlink_msg_wxbs_target_decode(&msg, &in_packet);
        bsq_waiting = true;
        last_send_mission_ms = 0;
        receive_bsq_ms = millis();
    }
}

void UMav_trans_target::send_mission_msg()
{
    //send target result 401;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_TARGET");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_target_send(
                    channel,
                    in_packet.start_lat,
                    in_packet.start_lng,
                    in_packet.start_alt,
                    in_packet.target_lat,
                    in_packet.target_lng,
                    in_packet.target_alt);
            }
        }
    }
}

void UMav_trans_target::handle_mission_msg(const mavlink_message_t &msg)
{
    //handle target cmd 404;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_TARGET_RESULT) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_TARGET_RESULT");
        // decode packet
        mavlink_msg_wxbs_target_result_decode(&msg, &out_packet);
        send_bsq_msg();
        bsq_waiting = false;
    }
}

void UMav_trans_target::send_bsq_msg()
{
    //send target result 404;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_TARGET_RESULT");

    mavlink_message_t msg;

    UNUSED_RESULT(mavlink_msg_wxbs_target_result_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &out_packet));

    copter.umav.send_bsq_message(&msg);
}

// ~~~~~~~~~~~~~~~~~~ WXBS_MISSION ~~~~~~~~~~~~~~~~~~
void UMav_trans_mission::update()
{
    if (!bsq_waiting) {return;}
    uint32_t tnow_ms = millis();
    if (tnow_ms - receive_bsq_ms > timeout_bsq_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "OUT WXBS_MISSION");
        out_packet.mission_ok = 0;
        send_bsq_msg();
        bsq_waiting = false;
    }

    if ((tnow_ms - last_send_mission_ms > repeat_time_ms) || (last_send_mission_ms == 0)) {
        send_mission_msg();
        last_send_mission_ms = tnow_ms;
    }
}

void UMav_trans_mission::handle_bsq_msg(const mavlink_message_t &msg)
{
    //handle relay cmd 405;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_MISSION) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_MISSION");
        // decode packet
        mavlink_msg_wxbs_mission_decode(&msg, &in_packet);
        bsq_waiting = true;
        last_send_mission_ms = 0;
        receive_bsq_ms = millis();
    }
}

void UMav_trans_mission::send_mission_msg()
{
    //send target result 405;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_MISSION");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_mission_send(
                    channel,
                    in_packet.group_id,
                    in_packet.uav_num,
                    in_packet.relay_id,
                    in_packet.package_number,
                    in_packet.count,
                    in_packet.lat,
                    in_packet.lon,
                    in_packet.alt);
            }
        }
    }
}

void UMav_trans_mission::handle_mission_msg(const mavlink_message_t &msg)
{
    //handle target cmd 406;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_MISSION_RESULT) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_MISSION_RESULT");
        // decode packet
        mavlink_msg_wxbs_mission_result_decode(&msg, &out_packet);
        send_bsq_msg();
        bsq_waiting = false;
    }
}

void UMav_trans_mission::send_bsq_msg()
{
    //send target result 406;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_MISSION_RESULT");

    mavlink_message_t msg;

    UNUSED_RESULT(mavlink_msg_wxbs_mission_result_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &out_packet));

    copter.umav.send_bsq_message(&msg);
}

// ~~~~~~~~~~~~~~~~~~ WXBS_RELAY_POSITION ~~~~~~~~~~~~~~~~~~
void UMav_trans_relay_positon::update()
{
    if (!bsq_waiting) {return;}
    uint32_t tnow_ms = millis();
    if (tnow_ms - receive_bsq_ms > timeout_bsq_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "OUT WXBS_RELAY_POSITION");
        out_packet.relay_position_ok = 0;
        send_bsq_msg();
        bsq_waiting = false;
    }

    if ((tnow_ms - last_send_mission_ms > repeat_time_ms) || (last_send_mission_ms == 0)) {
        send_mission_msg();
        last_send_mission_ms = tnow_ms;
    }
}

void UMav_trans_relay_positon::handle_bsq_msg(const mavlink_message_t &msg)
{
    //handle relay cmd 407;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_RELAY_POSITION) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_RELAY_POSITION");
        // decode packet
        mavlink_msg_wxbs_relay_position_decode(&msg, &in_packet);
        bsq_waiting = true;
        last_send_mission_ms = 0;
        receive_bsq_ms = millis();
    }
}

void UMav_trans_relay_positon::send_mission_msg()
{
    //send target result 407;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_RELAY_POSITION");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                mavlink_msg_wxbs_relay_position_send(
                    channel,
                    in_packet.group_id,
                    in_packet.uav_num,
                    in_packet.relay_id,
                    in_packet.lat,
                    in_packet.lon,
                    in_packet.alt);
            }
        }
    }
}

void UMav_trans_relay_positon::handle_mission_msg(const mavlink_message_t &msg)
{
    //handle target cmd 408;
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_RELAY_POSITION_RESULT) {
        gcs().send_text(MAV_SEVERITY_INFO, "Receive WXBS_RELAY_POSITION_RESULT");
        // decode packet
        mavlink_msg_wxbs_relay_position_result_decode(&msg, &out_packet);
        send_bsq_msg();
        bsq_waiting = false;
    }
}

void UMav_trans_relay_positon::send_bsq_msg()
{
    //send target result 408;
    gcs().send_text(MAV_SEVERITY_INFO, "Send WXBS_RELAY_POSITION_RESULT");

    mavlink_message_t msg;

    UNUSED_RESULT(mavlink_msg_wxbs_relay_position_result_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &out_packet));

    copter.umav.send_bsq_message(&msg);
}

