#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

void FD_DATA::handle_message_sn(const mavlink_message_t &msg)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "ID: %d", int(msg.msgid));
    if (msg.msgid == MAVLINK_MSG_ID_HXTS_SN) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_hxts_sn_t packet;
        mavlink_msg_hxts_sn_decode(&msg, &packet);
        if (int16_t(packet.cmd) == 1) {
            char sn[20];
            char uas[8];
            if (get_serial_number(sn, uas))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Get", sn);
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: UAS%s Get", uas);
                send_mav_serial_number();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN/UAS Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "SU: %s/%s", sn, uas);
            }
        }
        if (int16_t(packet.cmd) == 2) {
            char sn[21];
            char uas[9];
            memcpy(sn, packet.serial_number, sizeof(packet.serial_number));
            memcpy(uas, packet.uas_number, sizeof(packet.uas_number));
            if (set_serial_number(sn, uas)) 
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Set\n", sn);
                gcs().send_text(MAV_SEVERITY_INFO, "UAS: UAS%s Set\n", uas);
                if (get_serial_number(sn, uas))
                {
                    send_mav_serial_number();
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "SN/UAS Set Fail 2");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN/UAS Set Fail 1");
                gcs().send_text(MAV_SEVERITY_INFO, "SU: %s/%s", sn, uas);
            }
        }
    }
}

void FD_DATA::handle_message_command_long_sn(const mavlink_message_t &msg)
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
                        char uas[8];
                        if (get_serial_number(sn, uas)) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Get", sn);
                            send_mav_serial_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN/UAS Get Fail 1");
                            gcs().send_text(MAV_SEVERITY_INFO, "SU: %s/%s", sn, uas);
                        }
                    }
                    if (int16_t(packet.param1) == 2 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        char sn[21] = "Hello World 12345678";
                        char uas[9] = "12345678";
                        // hal.util->snprintf(sn, sizeof(local_data.serial_number), "TEST SN %d abcdefghij", int16_t(packet.param2));
                        // hal.util->snprintf(uas, sizeof(local_data.uas_number), "12345678", int16_t(packet.param2));
                        if (set_serial_number(sn, uas)) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %s Set", sn);
                            send_mav_serial_number();
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN/UAS Set Fail 1");
                            gcs().send_text(MAV_SEVERITY_INFO, "SU: %s/%s", sn, uas);
                        }
                    }
                    if (int16_t(packet.param1) == 99 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        read_serial_number();
                        gcs().send_text(MAV_SEVERITY_INFO, "SN refresh");
                    }
                }
                break;
            default:
                break;
        }
    }

}

void FD_DATA::send_mav_serial_number()
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    // mavlink_command_int_t command_int;
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_hxts_sn_send(
                    channel,
                    0,
                    3,
                    local_data.serial_number,
                    local_data.uas_number);
            }
        }
    }
}

bool FD_DATA::get_serial_number(char *serial_number, char *uas_number)
{
    if (local_data.serial_number[0] != 0) {
        memcpy(serial_number, local_data.serial_number, sizeof(local_data.serial_number));
        memcpy(uas_number, local_data.uas_number, sizeof(local_data.uas_number));
        return true;
    }

    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(serial_number, local_data.serial_number, sizeof(local_data.serial_number));
        memcpy(uas_number, local_data.uas_number, sizeof(local_data.uas_number));
        return true;
    }
    return false;
}

bool FD_DATA::read_serial_number()
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        return true;
    }
    return false;
}

bool FD_DATA::set_serial_number(char *serial_number, char *uas_number)
{
    if (_storage.read_block(&local_data, 0, sizeof(FD_DATA_T))) {
        memcpy(local_data.serial_number, serial_number, sizeof(local_data.serial_number));
        memcpy(local_data.uas_number, uas_number, sizeof(local_data.uas_number));
        return _storage.write_block(0, &local_data, sizeof(FD_DATA_T));
    }
    return false;
}
