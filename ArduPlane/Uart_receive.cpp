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
    bool type_change = false;
    if (control_status.type != uart_msg_LS_control._msg_1.content.msg.type) {
        type_change = true;
    }
    control_status.type = uart_msg_LS_control._msg_1.content.msg.type;
    switch (control_status.type) {
        case 0x1A:
        {
            control_status.cmd_speed = uart_msg_LS_control._msg_1.content.msg_0x1A.target_speed;
            control_status.cmd_pitch = uart_msg_LS_control._msg_1.content.msg_0x1A.target_pitch;
            control_status.cmd_roll = uart_msg_LS_control._msg_1.content.msg_0x1A.target_roll;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x1A.flight_status;
            control_status.last_cmd_ms = millis();
            if (type_change) {
                gcs().send_text(MAV_SEVERITY_INFO, "Ext CMD Angle");
            }
            update_log_angle();
            break;
        }
        case 0xFD:
        {
            control_status.cmd_speed = uart_msg_LS_control._msg_1.content.msg_0xFD.target_speed;
            control_status.cmd_pitch_rate = uart_msg_LS_control._msg_1.content.msg_0xFD.target_pitch_rate;
            control_status.cmd_yaw_rate = uart_msg_LS_control._msg_1.content.msg_0xFD.target_yaw_rate;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0xFD.flight_status;
            control_status.last_cmd_ms = millis();
            if (type_change) {
                gcs().send_text(MAV_SEVERITY_INFO, "Ext CMD PY-Rate");
            }
            update_log_rate();
            break;
        }
        case 0x3C:
        {
            control_status.cmd_speed = uart_msg_LS_control._msg_1.content.msg_0x3C.target_speed;
            control_status.cmd_alt = uart_msg_LS_control._msg_1.content.msg_0x3C.target_alt;
            control_status.cmd_roll = uart_msg_LS_control._msg_1.content.msg_0x3C.target_roll;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x3C.flight_status;
            control_status.last_cmd_ms = millis();
            if (type_change) {
                gcs().send_text(MAV_SEVERITY_INFO, "Ext CMD Spd-Hgt");
            }
            update_log_spd_hgt();
            break;
        }
        case 0x55:
        {
            control_status.cmd_loc.lng = (uart_msg_LS_control._msg_1.content.msg_0x55.wp_lng) * 1e7;
            control_status.cmd_loc.lat = (uart_msg_LS_control._msg_1.content.msg_0x55.wp_lat) * 1e7;
            control_status.cmd_loc.alt = uart_msg_LS_control._msg_1.content.msg_0x55.wp_alt * 100.f;
            control_status.cmd = uart_msg_LS_control._msg_1.content.msg_0x55.flight_status;
            control_status.last_cmd_ms = millis();
            if (type_change) {
                gcs().send_text(MAV_SEVERITY_INFO, "Ext CMD Waypoint");
            }
            update_log_waypoint();
            break;
        }
    }
}

void Uart::update_log_angle() {
    AP::logger().WriteStreaming("EANG",
                                "TimeUS,spd,pth,roll,cmd",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)control_status.cmd_speed,
                                (float)control_status.cmd_pitch,
                                (float)control_status.cmd_roll,
                                (float)control_status.cmd);
}

void Uart::update_log_rate() {
    AP::logger().WriteStreaming("ERAT",
                                "TimeUS,spd,prate,yrate,cmd",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)control_status.cmd_speed,
                                (float)control_status.cmd_pitch_rate,
                                (float)control_status.cmd_yaw_rate,
                                (float)control_status.cmd);
}

void Uart::update_log_spd_hgt() {
    AP::logger().WriteStreaming("ESH",
                                "TimeUS,spd,alt,roll,cmd",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)control_status.cmd_speed,
                                (float)control_status.cmd_alt,
                                (float)control_status.cmd_roll,
                                (float)control_status.cmd);
}

void Uart::update_log_waypoint() {
    AP::logger().WriteStreaming("EWP",
                                "TimeUS,lng,lat,alt,cmd",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)control_status.cmd_loc.lng,
                                (float)control_status.cmd_loc.lat,
                                (float)control_status.cmd_loc.alt,
                                (float)control_status.cmd);
}
