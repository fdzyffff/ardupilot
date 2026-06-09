/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Copter.h"

UGimbal::UGimbal()
{
    ;
}

// initialise
void UGimbal::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DYT, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UGimbal init");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "UGimbal init Fail");
        _port = nullptr;
    }
}

void UGimbal::update()
{
    read_uart();
    write_uart();
    check_alive();
    update_target();
    update_log();
}

void UGimbal::read_uart()
{
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        uart_msg_QD_S11.parse(temp);

        if (uart_msg_QD_S11._msg_1.updated) {
            _last_update_ms = millis();

            if ((uart_msg_QD_S11._msg_1.content.msg.track_status & 0x03) == 2) {
                float LSB = 360.f/65536.f;
                status.cam_yaw = (float)(uart_msg_QD_S11._msg_1.content.msg.cam_yaw)*LSB;
                status.cam_pitch = (float)(uart_msg_QD_S11._msg_1.content.msg.cam_pitch)*LSB;
                status.target_ms = millis();
            }

            uart_msg_QD_S11._msg_1.updated = false;   
        }
    }
}

void UGimbal::write_uart()
{
    // send_0x11();
    // send_0x22();
}

void UGimbal::check_alive()
{
    if (_last_update_ms < 10000) {return;}
    if (millis() - _last_update_ms > 2000) {
        if (_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal disconnect");
        }
        _alive = false;
    } else {
        if (!_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal connect");
        }
        _alive = true;
    }
}

void UGimbal::update_target()
{
    if (status.target_ms < 10000) {return;}
    if (millis() - status.target_ms > 2000) {
        if (status.have_target) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal Target lost");
        }
        status.have_target = false;
    } else {
        if (!status.have_target) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal Target aquire");
        }
        status.have_target = true;
    }
}

void UGimbal::update_log()
{
    if (!_alive) {return;}
    uint32_t now_ms = millis();
    if (now_ms - _last_log_ms < 500) {return;}

    _last_log_ms = now_ms;

    // AP::logger().WriteStreaming("UWGT",
    //                             "TimeUS,FRONT,LEFT,RIGHT",
    //                             "s---",
    //                             "F---",
    //                             "Qfff",
    //                             AP_HAL::micros64(),
    //                             (float)hxts_hy_weight_packet.Front,
    //                             (float)hxts_hy_weight_packet.LEFT,
    //                             (float)hxts_hy_weight_packet.RIGHT);
}

// void UGimbal::send_reply(uint8_t cmd_type)
// {
//     if (get_port() == nullptr) {return;}
//     uart_msg_reply._msg_1.content.msg.header.head_1 = uart_msg_reply.PREAMBLE1;
//     uart_msg_reply._msg_1.content.msg.header.head_2 = uart_msg_reply.PREAMBLE2;
//     uart_msg_reply._msg_1.content.msg.length = uart_msg_reply._msg_1.length;
//     uart_msg_reply._msg_1.content.msg.cmd_type = cmd_type;
//     uart_msg_reply.make_sum();
//     get_port()->write(uart_msg_reply._msg_1.content.data, sizeof(uart_msg_reply._msg_1.content.data));
// }
