#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

bool FD_DATA::get_runtime_flying(uint32_t& runtime_flying)
{
    FD_DATA_T fd_data_t;
    if (_storage.read_block(&fd_data_t, 0, sizeof(FD_DATA_T))) {
        runtime_flying = fd_data_t.runtime_flying;
        return true;
    }
    return false;
}

bool FD_DATA::reset_runtime_flying()
{
    FD_DATA_T fd_data_t;
    if (_storage.read_block(&fd_data_t, 0, sizeof(FD_DATA_T))) {
        fd_data_t.runtime_flying = 0;
        return _storage.write_block(0, &fd_data_t, sizeof(FD_DATA_T));
    }
    return false;
}

void FD_DATA::update_flying_s()
{
    static uint32_t _last_update_ms = AP_HAL::millis();
    if (AP_HAL::millis() - _last_update_ms < 5000) {return;}
    _last_update_ms = AP_HAL::millis();
    if (_is_flying) {
        uint32_t t_ms = AP_HAL::millis() - _last_flying_ms;
        uint32_t dt_s = t_ms/1000;
        _last_flying_ms = AP_HAL::millis() - t_ms%1000;

        FD_DATA_T fd_data_t;
        if (_storage.read_block(&fd_data_t, 0, sizeof(FD_DATA_T))) {
            fd_data_t.runtime_flying = fd_data_t.runtime_flying + dt_s;
            _storage.write_block(0, &fd_data_t, sizeof(FD_DATA_T));
        }
    } else {
        _last_flying_ms = AP_HAL::millis();
    }
}

void FD_DATA::set_is_flying(bool in)
{
    _is_flying = in;
}

void FD_DATA::handle_message_rt(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_HXTS_RT) {
        mavlink_hxts_rt_t packet;
        mavlink_msg_hxts_rt_decode(&msg, &packet);
        if (int16_t(packet.cmd) == 1) {
            uint32_t rt = 0;
            if(get_runtime_flying(rt))
            {
                gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                send_mav_runtime_flying(rt);
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
                if(get_runtime_flying(rt))
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "RT: %d", int(rt));
                    send_mav_runtime_flying(rt);
                }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "RT reset Fail");
            }
        }
    }
}

void FD_DATA::handle_message_command_long_rt(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_2:
                {
                    if (int16_t(packet.param1) == 1 && int16_t(packet.param5) == 150 && int16_t(packet.param6) == 1079 && int16_t(packet.param7) == 1500) {
                        uint32_t rt = 0;
                        if(get_runtime_flying(rt))
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

void FD_DATA::send_mav_runtime_flying(uint32_t runtime_flying)
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_hxts_rt_send(
                    channel,
                    0,
                    3,
                    runtime_flying);
            }
        }
    }
}
