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

void Uart::set_target_angle(float gimbal_yaw, float gimbal_pitch)
{
    send_status.gimbal_yaw = gimbal_yaw;
    send_status.gimbal_pitch = gimbal_pitch;
}

void Uart::set_target_loc(Location& loc_in)
{
    send_status.target_loc = loc_in;
}

void Uart::pack_status()
{
    if (millis() - send_status.last_status_send_ms > 10) {
        send_status.last_status_send_ms = millis();
    } else {
        return;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return;
    }

    Vector3f pos_ned;
    if ( !AP::ahrs().get_relative_position_NED_home(pos_ned) ) {
        pos_ned.zero();
    }
    Vector3f vel_ned;
    if ( !AP::ahrs().get_velocity_NED(vel_ned) ) {
        vel_ned.zero();
    }


    double target_lng = 0.0;
    double target_lat = 0.0;
    float target_alt = 0.0;

    if (send_status.target_loc.lng != 0 && send_status.target_loc.lat != 0) {
        target_lng = ((double)send_status.target_loc.lng) * 1e-7;
        target_lat = ((double)send_status.target_loc.lat) * 1e-7;
        int32_t tmp_alt;
        if (send_status.target_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            target_alt = ((float)tmp_alt) * 0.01f;
        }
    }

    double current_lng = 0.0;
    double current_lat = 0.0;
    float current_alt = 0.0;
    if (current_loc.lng != 0 && current_loc.lat != 0) {
        current_lng = ((double)current_loc.lng) * 1e-7;
        current_lat = ((double)current_loc.lat) * 1e-7;
        int32_t tmp_alt;
        current_lng = ((double)current_loc.lng) * 1e-7;
        if (current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            current_alt = ((float)tmp_alt) * 0.01f;
        }
    }

    float airspeed = 0.0f;
    if (AP::ahrs().airspeed_estimate(airspeed)) {
        ;
    }
    uart_msg_LS_status._msg_1.content.msg.run_time = ((float)AP_HAL::millis()) * 0.001f;
    uart_msg_LS_status._msg_1.content.msg.gimbal_pitch = send_status.gimbal_pitch;
    uart_msg_LS_status._msg_1.content.msg.gimbal_yaw = send_status.gimbal_yaw;
    uart_msg_LS_status._msg_1.content.msg.target_x = 0;
    uart_msg_LS_status._msg_1.content.msg.target_y = 0;
    uart_msg_LS_status._msg_1.content.msg.target_lng = target_lng;
    uart_msg_LS_status._msg_1.content.msg.target_lat = target_lat;
    uart_msg_LS_status._msg_1.content.msg.target_alt = target_alt;
    uart_msg_LS_status._msg_1.content.msg.current_lng = current_lng;
    uart_msg_LS_status._msg_1.content.msg.current_lat = current_lat;
    uart_msg_LS_status._msg_1.content.msg.vel_n = vel_ned.x;
    uart_msg_LS_status._msg_1.content.msg.vel_e = vel_ned.y;
    uart_msg_LS_status._msg_1.content.msg.vel_d = vel_ned.z;
    uart_msg_LS_status._msg_1.content.msg.roll = ((float)AP::ahrs().roll_sensor * 0.01f);
    uart_msg_LS_status._msg_1.content.msg.pitch = ((float)AP::ahrs().pitch_sensor * 0.01f);
    uart_msg_LS_status._msg_1.content.msg.yaw = ((float)AP::ahrs().yaw_sensor * 0.01f);
    uart_msg_LS_status._msg_1.content.msg.air_speed = airspeed;
    uart_msg_LS_status._msg_1.content.msg.yaw_rate = degrees(AP::ahrs().get_yaw_rate_earth());
    uart_msg_LS_status._msg_1.content.msg.pos_x = pos_ned.x;
    uart_msg_LS_status._msg_1.content.msg.pos_y = pos_ned.y;
    uart_msg_LS_status._msg_1.content.msg.pos_z = pos_ned.z;
    uart_msg_LS_status._msg_1.content.msg.current_alt = current_alt;

    uart_msg_LS_status.make_sum();
    uart_msg_LS_status.swap_message();

    if (get_port() != nullptr) {
        get_port()->write(uart_msg_LS_status._msg_1.content.data, sizeof(uart_msg_LS_status._msg_1.content.data));
        // get_port()->write(0xBE);
    }
}