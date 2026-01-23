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

Uart::Uart()
{
    ;
}

// initialise
void Uart::init()
{
    _port = nullptr;
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = plane.serial_manager.find_serial(AP_SerialManager::SerialProtocol_MISSION, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Uart init");
        return;
    }
}

void Uart::update()
{
    read_uart();
    write_uart();
}

void Uart::read_uart()
{
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();

        uart_msg_LS_control.parse(temp);
        handle_LS_control_receive();
    }

}

void Uart::write_uart()
{
    pack_status();
    if (get_port() != nullptr) {
        get_port()->write(uart_msg_LS_status._msg_1.content.data, sizeof(uart_msg_LS_status._msg_1.content.data));
    }
}

void Uart::handle_LS_control_receive()
{
    if (uart_msg_LS_control._msg_1.updated) {
        uart_msg_LS_control._msg_1.updated = false;

        if (uart_msg_LS_control._msg_1.content.msg.control_type == 0x02) {
            float cmd_speed = uart_msg_LS_control._msg_1.content.msg.cmd_speed;
            float cmd_pitch = uart_msg_LS_control._msg_1.content.msg.cmd_pitch;
            float cmd_roll = uart_msg_LS_control._msg_1.content.msg.cmd_roll;
            plane.uattack.set_external_cmd(cmd_speed, cmd_pitch, cmd_roll);
        }
    }
}

void Uart::pack_status()
{
    if (!plane.position_ok()) {return;}

    Vector3f pos_ned;
    if ( !AP::ahrs().get_relative_position_NED_home(pos_ned) ) {
        pos_ned.zero();
    }
    Vector3f vel_ned;
    if ( !AP::ahrs().get_velocity_NED(vel_ned) ) {
        vel_ned.zero();
    }
    float gimbal_yaw = 0.0f;
    float gimbal_pitch = 0.0f;
    float target_x = 0.0f;
    float target_y = 0.0f;
    if (plane.uattack._Target_ptr_cam_DYT != nullptr) {
        gimbal_yaw = (float)(plane.uattack._Target_ptr_cam_DYT->uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw) * 0.01f;
        gimbal_pitch = (float)(plane.uattack._Target_ptr_cam_DYT->uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch) * 0.01f;
        target_x = (float)(plane.uattack._Target_ptr_cam_DYT->uart_msg_DYT_telem._msg_1.content.msg.target_yaw) * 0.05f;
        target_y = (float)(plane.uattack._Target_ptr_cam_DYT->uart_msg_DYT_telem._msg_1.content.msg.target_pitch) * 0.05f;
    } else if (plane.uattack._Target_ptr_loc != nullptr) {
        if (plane.uattack._Target_ptr_loc->get_info(gimbal_yaw, gimbal_pitch)) {
            ;
        }
    }

    double target_lng = 0.0;
    double target_lat = 0.0;
    float target_alt = 0.0;
    if (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid()) {
        target_lng = ((double)plane.uattack._Target_ptr_loc->target_loc.lng) * 1e-7;
        target_lat = ((double)plane.uattack._Target_ptr_loc->target_loc.lat) * 1e-7;
        int32_t tmp_alt;
        if (plane.uattack._Target_ptr_loc->target_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            target_alt = ((float)tmp_alt) * 0.01f;
        }
    }

    double current_lng = 0.0;
    double current_lat = 0.0;
    float current_alt = 0.0;
    {
        current_lng = ((double)plane.current_loc.lng) * 1e-7;
        current_lat = ((double)plane.current_loc.lat) * 1e-7;
        int32_t tmp_alt;
        current_lng = ((double)plane.current_loc.lng) * 1e-7;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            current_alt = ((float)tmp_alt) * 0.01f;
        }
    }
    uart_msg_LS_status._msg_1.content.msg.run_time = ((float)AP_HAL::millis()) * 0.001f;
    uart_msg_LS_status._msg_1.content.msg.gimbal_pitch = gimbal_pitch;
    uart_msg_LS_status._msg_1.content.msg.gimbal_yaw = gimbal_yaw;
    uart_msg_LS_status._msg_1.content.msg.target_x = target_x;
    uart_msg_LS_status._msg_1.content.msg.target_y = target_y;
    uart_msg_LS_status._msg_1.content.msg.target_lng = target_lng;
    uart_msg_LS_status._msg_1.content.msg.target_lat = target_lat;
    uart_msg_LS_status._msg_1.content.msg.target_alt = target_alt;
    uart_msg_LS_status._msg_1.content.msg.current_lng = current_lng;
    uart_msg_LS_status._msg_1.content.msg.current_lat = current_lat;
    uart_msg_LS_status._msg_1.content.msg.vel_n = vel_ned.x;
    uart_msg_LS_status._msg_1.content.msg.vel_e = vel_ned.y;
    uart_msg_LS_status._msg_1.content.msg.vel_d = vel_ned.z;
    uart_msg_LS_status._msg_1.content.msg.roll = degrees(AP::ahrs().get_roll());
    uart_msg_LS_status._msg_1.content.msg.pitch = degrees(AP::ahrs().get_pitch());
    uart_msg_LS_status._msg_1.content.msg.yaw = degrees(AP::ahrs().get_yaw());
    uart_msg_LS_status._msg_1.content.msg.air_speed = plane.airspeed.get_airspeed();
    uart_msg_LS_status._msg_1.content.msg.yaw_rate = degrees(AP::ahrs().get_yaw_rate_earth());
    uart_msg_LS_status._msg_1.content.msg.pos_x = pos_ned.x;
    uart_msg_LS_status._msg_1.content.msg.pos_y = pos_ned.y;
    uart_msg_LS_status._msg_1.content.msg.pos_z = pos_ned.z;
    uart_msg_LS_status._msg_1.content.msg.current_alt = current_alt;
}
