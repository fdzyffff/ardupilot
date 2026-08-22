#include "FD_DATA.h"

void FD_DATA::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ZFJL_SN:
        handle_message_sn(chan, msg);
        break;
    case MAVLINK_MSG_ID_ZFJL_UAS:
        handle_message_uas(chan, msg);
        break;
    case MAVLINK_MSG_ID_ZFJL_RT:
        handle_message_rt(chan, msg);
        break;
    case MAVLINK_MSG_ID_ZFJL_GCS_HEARTBEAT:
        handle_message_gcs_heartbeat(msg);
        break;
    default:
        break;
    }
}

void FD_DATA::handle_message_sn(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_zfjl_sn_t packet{};
    mavlink_msg_zfjl_sn_decode(&msg, &packet);
    bool success = true;
    if (packet.cmd == 2) {
        success = set_serial_number(packet.serial_number, sizeof(packet.serial_number));
    }
    if ((packet.cmd == 1 || packet.cmd == 2) && success) {
        send_zfjl_sn(chan);
    }
}

void FD_DATA::handle_message_uas(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_zfjl_uas_t packet{};
    mavlink_msg_zfjl_uas_decode(&msg, &packet);
    bool success = true;
    if (packet.cmd == 2) {
        success = set_uas_number(packet.uas_number, sizeof(packet.uas_number));
    }
    if ((packet.cmd == 1 || packet.cmd == 2) && success) {
        send_zfjl_uas(chan);
    }
}

void FD_DATA::handle_message_rt(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_zfjl_rt_t packet{};
    mavlink_msg_zfjl_rt_decode(&msg, &packet);
    bool success = true;
    if (packet.cmd == 2) {
        success = reset_runtime_flying();
    }
    if ((packet.cmd == 1 || packet.cmd == 2) && success) {
        send_zfjl_rt(chan);
    }
}

void FD_DATA::handle_message_gcs_heartbeat(const mavlink_message_t &msg)
{
    mavlink_msg_zfjl_gcs_heartbeat_decode(&msg, &zfjl_gcs_heartbeat_packet);
    _last_gcs_heartbeat_ms = AP_HAL::millis();
    zfjl_uav_heartbeat_packet.rc_latitude = zfjl_gcs_heartbeat_packet.latitude;
    zfjl_uav_heartbeat_packet.rc_longitude = zfjl_gcs_heartbeat_packet.longitude;
    zfjl_uav_heartbeat_packet.rc_altitude = zfjl_gcs_heartbeat_packet.altitude;
}

void FD_DATA::send_zfjl_sn(mavlink_channel_t chan)
{
    char serial[IDENTITY_TEXT_SIZE]{};
    if (get_serial_number(serial)) {
        mavlink_msg_zfjl_sn_send(chan, 3, serial);
    }
}

void FD_DATA::send_zfjl_rt(mavlink_channel_t chan)
{
    uint32_t runtime = 0;
    if (get_runtime_flying(runtime)) {
        mavlink_msg_zfjl_rt_send(chan, 3, runtime);
    }
}

void FD_DATA::send_zfjl_uas(mavlink_channel_t chan)
{
    char uas[IDENTITY_TEXT_SIZE]{};
    if (get_uas_number(uas)) {
        mavlink_msg_zfjl_uas_send(chan, 3, uas);
    }
}

void FD_DATA::send_zfjl_uav_heartbeat(mavlink_channel_t chan)
{
    zfjl_uav_heartbeat_packet.type = uav_type.get();
    if (_last_gcs_heartbeat_ms == 0 ||
        AP_HAL::millis() - _last_gcs_heartbeat_ms > 10000U) {
        zfjl_uav_heartbeat_packet.rc_latitude = 0;
        zfjl_uav_heartbeat_packet.rc_longitude = 0;
        zfjl_uav_heartbeat_packet.rc_altitude = 0;
    }
    mavlink_msg_zfjl_uav_heartbeat_send_struct(chan, &zfjl_uav_heartbeat_packet);
}
