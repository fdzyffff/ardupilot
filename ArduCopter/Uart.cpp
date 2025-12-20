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
        uint8_t temp = get_port()->read();
        uart_msg_0728_p3.parse(temp);
        if (uart_msg_0728_p3._msg_1.updated) {
            gcs().send_text(MAV_SEVERITY_INFO, "-> New Target WPs <-");
            int32_t lat_1 = uart_msg_0728_p3._msg_1.content.msg.wp_lat_1;
            int32_t lng_1 = uart_msg_0728_p3._msg_1.content.msg.wp_lng_1;
            int32_t alt_1 = ((int32_t)uart_msg_0728_p3._msg_1.content.msg.wp_alt_1)*100;
            Location temp_loc_1 = Location(lat_1, lng_1, alt_1, Location::AltFrame::ABSOLUTE);
            int32_t lat_2 = uart_msg_0728_p3._msg_1.content.msg.wp_lat_2;
            int32_t lng_2 = uart_msg_0728_p3._msg_1.content.msg.wp_lng_2;
            int32_t alt_2 = ((int32_t)uart_msg_0728_p3._msg_1.content.msg.wp_alt_2)*100;
            Location temp_loc_2 = Location(lat_2, lng_2, alt_2, Location::AltFrame::ABSOLUTE);
            uart_msg_0728_p3._msg_1.updated = false;

            if (copter.flightmode->mode_number() == Mode::Number::MISSION) {
                copter.mode_mission.set_loc(temp_loc_1, temp_loc_2);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "-> Err, Not in MISSION Mode <-");
            }
        }
    }
}

void Uart::write_uart()
{
    if (copter.flightmode->mode_number() == Mode::Number::MISSION) {
        send_0728_p1();
        // send_0728_p2();
    }
}

void Uart::send_0728_p1()
{
    // check send condition
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    if (now - last_ms < 833) {
        return;
    }
    last_ms = now;

    if (get_port() == nullptr) {return;}
    uart_msg_0728_p1._msg_1.content.msg.header.head_1 = uart_msg_0728_p1.PREAMBLE1;
    uart_msg_0728_p1._msg_1.content.msg.header.head_2 = uart_msg_0728_p1.PREAMBLE2;
    uart_msg_0728_p1._msg_1.content.msg.length = 0x1F;
    uart_msg_0728_p1._msg_1.content.msg.count += 1;
    uart_msg_0728_p1._msg_1.content.msg.recieve_id = copter.g.sysid_this_mav.get();
    uart_msg_0728_p1._msg_1.content.msg.command_id = 0x04;

    uart_msg_0728_p1._msg_1.content.msg.wp_lng = copter.current_loc.lng;
    uart_msg_0728_p1._msg_1.content.msg.wp_lat = copter.current_loc.lat;
    uart_msg_0728_p1._msg_1.content.msg.wp_alt = (uint16_t)(copter.current_loc.alt/100);
    uart_msg_0728_p1._msg_1.content.msg.wp_alt = (uint16_t)(AP::ahrs().groundspeed());

    uart_msg_0728_p1._msg_1.content.msg.pitch = (int32_t)(wrap_180_cd(degrees(AP::ahrs().get_pitch())*100.f));
    uart_msg_0728_p1._msg_1.content.msg.roll = (int32_t)(wrap_180_cd(degrees(AP::ahrs().get_roll())*100.f));
    uart_msg_0728_p1._msg_1.content.msg.yaw = (int32_t)(wrap_360_cd(degrees(AP::ahrs().get_yaw())*100.f));

    uart_msg_0728_p1.make_sum();
    get_port()->write(uart_msg_0728_p1._msg_1.content.data, sizeof(uart_msg_0728_p1._msg_1.content.data));
}

void Uart::send_0728_p2()
{
    // check send condition
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    if (now - last_ms < 500) {
        return;
    }
    last_ms = now;

    if (get_port() == nullptr) {return;}
    uart_msg_0728_p2._msg_1.content.msg.header.head_1 = uart_msg_0728_p2.PREAMBLE1;
    uart_msg_0728_p2._msg_1.content.msg.header.head_2 = uart_msg_0728_p2.PREAMBLE2;
    uart_msg_0728_p2._msg_1.content.msg.length = 0x17;
    uart_msg_0728_p2._msg_1.content.msg.recieve_id = copter.g.sysid_this_mav.get();
    uart_msg_0728_p2._msg_1.content.msg.command_id = 0x03;

    uart_msg_0728_p2._msg_1.content.msg.wp_lng = copter.mode_mission.get_target_loc().lng;
    uart_msg_0728_p2._msg_1.content.msg.wp_lat = copter.mode_mission.get_target_loc().lat;
    uart_msg_0728_p2._msg_1.content.msg.wp_alt = (uint16_t)(copter.mode_mission.get_target_loc().alt/100);
    uart_msg_0728_p2._msg_1.content.msg.wp_alt = (uint16_t)(copter.mode_mission.get_target_speed());

    uint8_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t second_ms = 0;

    uart_msg_0728_p2._msg_1.content.msg.year = year;
    uart_msg_0728_p2._msg_1.content.msg.month = month;
    uart_msg_0728_p2._msg_1.content.msg.day = day;
    uart_msg_0728_p2._msg_1.content.msg.hour = hour;
    uart_msg_0728_p2._msg_1.content.msg.minute = minute;
    uart_msg_0728_p2._msg_1.content.msg.second = second;
    uart_msg_0728_p2._msg_1.content.msg.second_ms = second_ms;

    uart_msg_0728_p2.make_sum();
    get_port()->write(uart_msg_0728_p2._msg_1.content.data, sizeof(uart_msg_0728_p2._msg_1.content.data));
}
