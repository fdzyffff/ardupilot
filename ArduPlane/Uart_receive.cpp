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


#include "Plane.h"

void Uart::handle_LS_control() {
    control_status.type = uart_msg_LS_control._msg_1.content.msg.type;
    switch (control_status.type) {
        case 0x1A:
        {
            control_status.cmd_speed = uart_msg_LS_control._msg_1.content.msg_0x1A.target_speed;
            control_status.cmd_pitch = uart_msg_LS_control._msg_1.content.msg_0x1A.target_pitch;
            control_status.cmd_roll = uart_msg_LS_control._msg_1.content.msg_0x1A.target_roll;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x1A.flight_status;
            control_status.last_cmd_ms = millis();
            break;
        }
        case 0x3C:
        {
            control_status.cmd_speed = uart_msg_LS_control._msg_1.content.msg_0x3C.target_speed;
            control_status.cmd_alt = uart_msg_LS_control._msg_1.content.msg_0x3C.target_alt;
            control_status.cmd_roll = uart_msg_LS_control._msg_1.content.msg_0x3C.target_roll;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x3C.flight_status;
            control_status.last_cmd_ms = millis();
            break;
        }
        case 0x55:
        {
            control_status.cmd_loc.lng = (uart_msg_LS_control._msg_1.content.msg_0x55.wp_lng) * 1e7;
            control_status.cmd_loc.lat = (uart_msg_LS_control._msg_1.content.msg_0x55.wp_lat) * 1e7;
            control_status.cmd_loc.alt = uart_msg_LS_control._msg_1.content.msg_0x55.wp_alt * 100.f;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x55.flight_status;
            control_status.last_cmd_ms = millis();
            break;
        }
    }
}

void Uart::update_status() {
    if (millis() - control_status.last_cmd_ms > 2000) {
        if (control_status.valid) {
            control_status.valid = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Lost External CMD");
        }
    } else {
        if (!control_status.valid) {
            control_status.valid = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Get External CMD");
        }
    }
}

