#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

bool FD_DATA::get_serial_number(uint32_t& serial_number)
{
    FD_DATA_T fd_data_t;
    if (_storage.read_block(&fd_data_t, 0, sizeof(FD_DATA_T))) {
        serial_number = fd_data_t.serial_number;
        return true;
    }
    return false;
}

bool FD_DATA::set_serial_number(uint32_t serial_number)
{
    FD_DATA_T fd_data_t;
    if (_storage.read_block(&fd_data_t, 0, sizeof(FD_DATA_T))) {
        fd_data_t.serial_number = serial_number;
        return _storage.write_block(0, &fd_data_t, sizeof(FD_DATA_T));
    }
    return false;
}

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
            uint32_t sn = 0;
            if(get_serial_number(sn))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %d Get", int(sn));
                send_mav_serial_number(sn);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %d", int(sn));
            }
        }
        if (int16_t(packet.cmd) == 2) {
            uint32_t sn = packet.serialnumber;
            if (set_serial_number(sn)) 
            {
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %d Set", int(sn));
                if(get_serial_number(sn))
                {
                    send_mav_serial_number(sn);
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                gcs().send_text(MAV_SEVERITY_INFO, "SN: %d", int(sn));
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
                        uint32_t sn = 0;
                        if(get_serial_number(sn))
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %d Get", int(sn));
                            send_mav_serial_number(sn);
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %d", int(sn));
                        }
                    }
                    if (int16_t(packet.param1) == 2 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        uint16_t serial_number_part1 = (uint16_t)packet.param2;
                        uint16_t serial_number_part2 = (uint16_t)packet.param3;
                        uint32_t sn = ((uint32_t)serial_number_part1<<16) + (uint32_t)serial_number_part2;
                        if (set_serial_number(sn)) 
                        {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %d Set", int(sn));
                            send_mav_serial_number(sn);
                        } else {
                            gcs().send_text(MAV_SEVERITY_INFO, "SN Set Fail");
                            gcs().send_text(MAV_SEVERITY_INFO, "SN: %d", int(sn));
                        }
                    }
                    if (int16_t(packet.param1) == 99 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        send_mav_serial_number_get();
                        gcs().send_text(MAV_SEVERITY_INFO, "SN get test");
                    }
                }
                break;
            default:
                break;
        }
    }

}

void FD_DATA::send_mav_serial_number(uint32_t serial_number)
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
                    serial_number);
            }
        }
    }
}

void FD_DATA::send_mav_serial_number_get()
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
                    1,
                    0);
            }
        }
    }
}
