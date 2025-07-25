#include "Copter.h"

UMission::UMission()
{

}

// initialise
void UMission::init()
{
    _valid = false;
    _uart_control.init();
    _uart_control.get_msg_status().set_enable();
    _uart_control.get_msg_control().set_enable();
    _last_ms = 0;
    _last_log_ms = 0;
    _last_mav_ms = 0;
}

// called at 100 Hz
void UMission::update()
{
    update_uart_read();
    update_uart_send();
    update_valid();
    update_log();
    update_mav();
}

void UMission::update_uart_read()
{
    if (_uart_control.initialized() && _uart_control.get_port() != nullptr) {
        while (_uart_control.get_port()->available() > 0) {
            uint8_t temp = _uart_control.get_port()->read();
            _uart_control.read(temp);
            if (_uart_control.get_msg_control()._msg_1.updated) {
                handle_msg_control();
            }
        }
    }
}

void UMission::update_uart_send()
{
    // send msg status
    send_status();
}

void UMission::update_valid()
{
    if (_last_ms == 0) {
        _valid = false;
        return;
    }

    if (millis() - _last_ms > 2000) {
        if (_valid) {
            _valid = false;
            gcs().send_text(MAV_SEVERITY_INFO, "[Danji] Lost control");
        }
    } else {
        if (!_valid) {
            _valid = true;
            gcs().send_text(MAV_SEVERITY_INFO, "[Danji] Get control");
        }
    }
}

void UMission::handle_msg_control()
{
    FD1_msg_control &tmp_msg = _uart_control.get_msg_control();
    if (tmp_msg._msg_1.updated) {
        _control_corr_bfy = (float)(tmp_msg._msg_1.content.msg.corr_bfy)*0.01f;
        _control_corr_bfz = (float)(tmp_msg._msg_1.content.msg.corr_bfz)*0.01f;
        tmp_msg._msg_1.updated = false;
        _last_ms = millis();
    }

    static uint32_t _last_print_ms = millis();
    if (millis() - _last_print_ms > 2000) {
        _last_print_ms = millis();
        if (copter.g2.user_parameters.print.get() > 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "_control_corr_bfy %f", _control_corr_bfy);
            gcs().send_text(MAV_SEVERITY_INFO, "_control_corr_bfz %f", _control_corr_bfz);
        }
    }
}

void UMission::send_status()
{
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms < 5) {return;}
    _last_send_ms = millis();

    FD1_msg_status &tmp_msg = _uart_control.get_msg_status();

    //uint32_t
    tmp_msg._msg_1.content.msg.sys_time = millis();
    //uint8_t
    tmp_msg._msg_1.content.msg.gps_ok = copter.position_ok();
    //int32_t
    tmp_msg._msg_1.content.msg.lng = copter.position_ok()? copter.current_loc.lng/10 : 0;
    //int32_t
    tmp_msg._msg_1.content.msg.lat = copter.position_ok()? copter.current_loc.lat/10 : 0;

    int32_t tmp_alt = 0;
    if (copter.position_ok()) {
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            tmp_alt = tmp_alt/10;
        }
    }

    if (tmp_alt < 0) {
        tmp_alt = 0;
    }
    //uint16_t
    tmp_msg._msg_1.content.msg.alt = (uint16_t)(tmp_alt/10);

    int32_t home_lng = 0;
    int32_t home_lat = 0;
    int32_t home_alt = 0;
    if (copter.ahrs.home_is_set()) {
        home_lng = copter.ahrs.get_home().lng/10;
        home_lat = copter.ahrs.get_home().lat/10;
        if (copter.ahrs.get_home().get_alt_cm(Location::AltFrame::ABSOLUTE, home_alt)) {
            home_alt = home_alt/10;
                if (home_alt < 0) {
                    home_alt = 0;
                }
        }
    }
    //int32_t
    tmp_msg._msg_1.content.msg.launch_lng = home_lng;
    //int32_t
    tmp_msg._msg_1.content.msg.launch_lng = home_lat;
    //uint16_t
    tmp_msg._msg_1.content.msg.launch_alt = (uint16_t)home_alt;

    tmp_msg.make_sum();

    tmp_msg._msg_1.need_send = true;

    _uart_control.write();
}

// for test purpose
void UMission::handle_msg(const mavlink_message_t &msg)
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
                    _last_ms = millis();
                    _control_corr_bfy = packet.param1*0.01f;
                    _control_corr_bfz = packet.param2*0.01f;
                }
                break;
            default:
                break;
        }
    }
}

void UMission::update_log()
{
    if (millis() - _last_log_ms < 100) {return;}
    _last_log_ms = millis();
    AP::logger().WriteStreaming("UARL",
                                "TimeUS,valid,bfy,bfz",
                                "s---",
                                "F---",
                                "Qfff",
                                AP_HAL::micros64(),
                                (float)_valid,
                                (float)_control_corr_bfy,
                                (float)_control_corr_bfz);

}


void UMission::update_mav()
{
    if (millis() - _last_mav_ms < 500) {return;}
    _last_mav_ms = millis();
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_command_long_send(
                    channel,
                    0,
                    0,
                    MAV_CMD_USER_1,
                    _control_corr_bfy,
                    _control_corr_bfz,
                    0, 0, 0, 0, 0, 0);
            }
        }
    }
}
