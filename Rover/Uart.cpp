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


#include "Rover.h"

Uart::Uart()
{
    ;
}

// initialise
void Uart::init()
{
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UART, 0);
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
        // uint8_t temp = get_port()->read();
        ;
    }
}

void Uart::write_uart()
{
    if (get_port() == nullptr) {return;}
    _msg.msg.header = PREAMBLE1;
    _msg.msg.bearing = cal_bearing();
    _msg.msg.speed = cal_speed();
    _msg.msg.accumulate++;
    get_port()->write(_msg.data, sizeof(_msg.data));
    if (rover.g2.user_uart_print.get() == 1 && _msg.msg.accumulate%50 == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "B:%d, S:%d",_msg.msg.bearing, _msg.msg.speed);
    }
}

uint8_t Uart::cal_bearing()
{
    float b1 = rover.control_mode->wp_bearing();
    float b2 = rover.control_mode->nav_bearing();
    float float_bearing = 0.0f;
    if (is_zero(b1) && is_zero(b2)) {
        float_bearing = 0.0f;
    }
    if (is_zero(b1) && !is_zero(b2)) {
        float_bearing = b2;
    }
    if (!is_zero(b1) && is_zero(b2)) {
        float_bearing = b1;
    }
    if (!is_zero(b1) && !is_zero(b2)) {
        float_bearing = b1;
    }

    if (is_zero(float_bearing)) {
        float_bearing = 90.f;
    } else {
        float_bearing = float_bearing - degrees(rover.ahrs.get_yaw());
        float_bearing = wrap_180(float_bearing);
        float_bearing = constrain_float(float_bearing + 90.0f, 0.0f, 180.0f);
    }
    return (uint8_t)float_bearing;
}

uint8_t Uart::cal_speed()
{
    float float_speed = constrain_float(rover.control_mode->get_desired_speed() + 100.f, 0.0f, 200.0f);
    return (uint8_t)float_speed;
}
