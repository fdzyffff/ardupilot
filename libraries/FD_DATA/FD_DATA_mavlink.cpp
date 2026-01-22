#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

void FD_DATA::handle_message(const mavlink_message_t &msg)
{
    handle_message_sn(msg);
    handle_message_uas(msg);
    handle_message_rt(msg);
    handle_message_gcs_heartbeat(msg);
    handle_message_command_long(msg);
}

void FD_DATA::handle_message_sn(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_ZFJL_SN) {
        mavlink_zfjl_sn_t packet;
        mavlink_msg_zfjl_sn_decode(&msg, &packet);
        if (int16_t(packet.cmd) == 1) {
            char sn[20];
            if (get_serial_number(sn))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Get", sn);
                send_mav_serial_number();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s", sn);
            }
        }
        if (int16_t(packet.cmd) == 2) {
            char sn[20];
            memcpy(sn, packet.serial_number, sizeof(packet.serial_number));
            if (set_serial_number(sn)) 
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Set", sn);
                if (get_serial_number(sn))
                {
                    send_mav_serial_number();
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s", sn);
            }
        }
    }
}

void FD_DATA::handle_message_uas(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_ZFJL_UAS) {
        mavlink_zfjl_uas_t packet;
        mavlink_msg_zfjl_uas_decode(&msg, &packet);
        if (int16_t(packet.cmd) == 1) {
            char uas[20];
            if (get_uas_number(uas))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s Get", uas);
                send_mav_uas_number();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "UAS Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s", uas);
            }
        }
        if (int16_t(packet.cmd) == 2) {
            char uas[20];
            memcpy(uas, packet.uas_number, sizeof(packet.uas_number));
            if (set_uas_number(uas)) 
            {
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s Set", uas);
                if (get_uas_number(uas))
                {
                    send_mav_uas_number();
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "UAS Set Fail");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "UAS Set Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s", uas);
            }
        }
    }
}

void FD_DATA::handle_message_rt(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_ZFJL_RT) {
        mavlink_zfjl_rt_t packet;
        mavlink_msg_zfjl_rt_decode(&msg, &packet);
        if (int16_t(packet.cmd) == 1) {
            uint32_t rt = 0;
            if (get_runtime_flying(rt))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                send_mav_runtime_flying();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "RT Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
            }
        }
        if (int16_t(packet.cmd) == 2) {
            if (reset_runtime_flying()) 
            {
                gcs().send_text(MAV_SEVERITY_INFO, "RT: reset");
                uint32_t rt = 0;
                if (get_runtime_flying(rt))
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                    send_mav_runtime_flying();
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "RT reset Fail");
            }
        }
    }
}

void FD_DATA::handle_message_gcs_heartbeat(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_ZFJL_GCS_HEARTBEAT) {
        mavlink_msg_zfjl_gcs_heartbeat_decode(&msg, &zfjl_gcs_heartbeat_packet);
    }
    zfjl_uav_heartbeat_packet.rc_latitude = zfjl_gcs_heartbeat_packet.latitude;
    zfjl_uav_heartbeat_packet.rc_longitude = zfjl_gcs_heartbeat_packet.longitude;
    zfjl_uav_heartbeat_packet.rc_altitude = zfjl_gcs_heartbeat_packet.altitude;
}

void FD_DATA::handle_message_command_long(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                {
                    if (int16_t(packet.param1) == 1 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        char sn[20];
                        if (get_serial_number(sn))
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Get", sn);
                            send_mav_serial_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s", sn);
                        }
                    }
                    if (int16_t(packet.param1) == 2 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        char sn[20];
                        hal.util->snprintf(sn, sizeof(local_data.serial_number), "TEST SN %d", int16_t(packet.param2));
                        if (set_serial_number(sn)) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Set", sn);
                            send_mav_serial_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s", sn);
                        }
                    }
                    if (int16_t(packet.param1) == 99 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        send_mav_serial_number();
                        gcs().send_text(MAV_SEVERITY_INFO, "SN get test");
                    }
                }
                break;
            case MAV_CMD_USER_2:
                {
                    if (int16_t(packet.param1) == 1 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        char uas[20];
                        if (get_uas_number(uas))
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s Get", uas);
                            send_mav_uas_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s", uas);
                        }
                    }
                    if (int16_t(packet.param1) == 2 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        char uas[20];
                        hal.util->snprintf(uas, sizeof(local_data.uas_number), "TEST UAS %d", int16_t(packet.param2));
                        if (set_uas_number(uas)) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s Set", uas);
                            send_mav_uas_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS Set Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "UAS: %s", uas);
                        }
                    }
                    if (int16_t(packet.param1) == 99 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        send_mav_uas_number();
                        gcs().send_text(MAV_SEVERITY_INFO, "UAS get test");
                    }
                }
                break;
            case MAV_CMD_USER_3:
                {
                    if (int16_t(packet.param1) == 1 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        uint32_t rt = 0;
                        if (get_runtime_flying(rt))
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "RT Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                        }
                    }
                    if (int16_t(packet.param1) == 2 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        if (reset_runtime_flying()) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "RT: reset");
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "RT reset Fail");
                        }
                    }
                }
                break;
            default:
                break;
        }
    }
}


void FD_DATA::send_zfjl_sn(mavlink_channel_t chan)
{
    mavlink_msg_zfjl_sn_send(
        chan,
        3,
        local_data.serial_number);
}

void FD_DATA::send_zfjl_uas(mavlink_channel_t chan)
{
    mavlink_msg_zfjl_uas_send(
        chan,
        3,
        local_data.uas_number);
}

void FD_DATA::send_zfjl_gcs_heartbeat(mavlink_channel_t chan)
{
    zfjl_uav_heartbeat_packet.type = uav_type.get();
    mavlink_msg_zfjl_gcs_heartbeat_send_struct(
        chan,
        &zfjl_gcs_heartbeat_packet);
}

void FD_DATA::send_zfjl_uav_heartbeat(mavlink_channel_t chan)
{
    mavlink_msg_zfjl_uav_heartbeat_send_struct(
        chan,
        &zfjl_uav_heartbeat_packet);
}

void FD_DATA::send_mav_serial_number()
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    // mavlink_command_int_t command_int;
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                send_zfjl_sn(channel);
            }
        }
    }
}

void FD_DATA::send_mav_uas_number()
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    // mavlink_command_int_t command_int;
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                send_zfjl_uas(channel);
            }
        }
    }
}

void FD_DATA::send_mav_runtime_flying()
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_zfjl_rt_send(
                    channel,
                    3,
                    local_data.runtime_flying);
            }
        }
    }
}
