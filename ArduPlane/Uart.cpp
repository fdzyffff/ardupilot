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
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NET, 0);
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
        uart_msg_0x31.parse(temp);
        uart_msg_0x33.parse(temp);
        uart_msg_0x36.parse(temp);
        uart_msg_0x37.parse(temp);

        if (uart_msg_0x31._msg_1.updated) {
            send_reply(uart_msg_0x31._msg_1.content.msg.cmd_type);
            gcs().send_text(MAV_SEVERITY_INFO, "-> New Target WP <-");
            int32_t lat_in = (double)uart_msg_0x31._msg_1.content.msg.target_lat*uart_msg_0x31.SF_LAT*1e7;
            int32_t lng_in = (double)uart_msg_0x31._msg_1.content.msg.target_lng*uart_msg_0x31.SF_LNG*1e7;
            int32_t alt_in = (uart_msg_0x31._msg_1.content.msg.target_alt-1000)*100;
            Location temp_loc = Location(lat_in, lng_in, alt_in, Location::AltFrame::ABSOLUTE);
            // gcs().send_text(MAV_SEVERITY_INFO, "-> alt_type %d <-", uart_msg_0x31._msg_1.content.msg.alt_type);
            if (uart_msg_0x31._msg_1.content.msg.alt_type == 0) {
                temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABOVE_HOME);
            }
            if (uart_msg_0x31._msg_1.content.msg.alt_type == 1) {
                temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABSOLUTE);
            }
            if (uart_msg_0x31._msg_1.content.msg.alt_type == 2) {
                temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABOVE_TERRAIN);
            }
            if (uart_msg_0x31._msg_1.content.msg.alt_type == 3) {
                temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABSOLUTE);
            }

            float lat = lat_in;
            float lng = lng_in;
            float alt = alt_in*0.01f;
            // float spd = uart_msg_0x31._msg_1.content.msg.target_speed*0.1f;
            gcs().send_text(MAV_SEVERITY_INFO, "-> [%f, %f, %f] <-", lng, lat, alt);
            if (plane.control_mode == &plane.mode_guided) {
                ;
            } else if (plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND)) {
                gcs().send_text(MAV_SEVERITY_INFO, "-> In GUIDED <-");
                plane.control_mode->handle_guided_request(temp_loc);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Err, Can't to GUIDED");
            }
            uart_msg_0x31._msg_1.updated = false;
        }

        if (uart_msg_0x33._msg_1.updated) {
            send_reply(uart_msg_0x33._msg_1.content.msg.cmd_type);
            uart_msg_0x33._msg_1.updated = false;
        }

        if (uart_msg_0x36._msg_1.updated) {
            send_reply(uart_msg_0x36._msg_1.content.msg.cmd_type);
            uart_msg_0x36._msg_1.updated = false;
        }

        if (uart_msg_0x37._msg_1.updated) {
            send_reply(uart_msg_0x37._msg_1.content.msg.cmd_type);
            uart_msg_0x37._msg_1.updated = false;
        }
    }
}

void Uart::write_uart()
{
    // send_0x11();
    // send_0x22();
}

void Uart::send_reply(uint8_t cmd_type)
{
    if (get_port() == nullptr) {return;}
    uart_msg_reply._msg_1.content.msg.header.head_1 = uart_msg_reply.PREAMBLE1;
    uart_msg_reply._msg_1.content.msg.header.head_2 = uart_msg_reply.PREAMBLE2;
    uart_msg_reply._msg_1.content.msg.length = uart_msg_reply._msg_1.length;
    uart_msg_reply._msg_1.content.msg.cmd_type = cmd_type;
    uart_msg_reply.make_sum();
    get_port()->write(uart_msg_reply._msg_1.content.data, sizeof(uart_msg_reply._msg_1.content.data));
}

void Uart::send_0x11()
{
    // check send condition
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    if (now - last_ms < 200) {
        return;
    }
    last_ms = now;

    if (get_port() == nullptr) {return;}
    uart_msg_0x11._msg_1.content.msg.header.head_1 = uart_msg_0x11.PREAMBLE1;
    uart_msg_0x11._msg_1.content.msg.header.head_2 = uart_msg_0x11.PREAMBLE2;
    uart_msg_0x11._msg_1.content.msg.length = uart_msg_0x11._msg_1.length;
    uart_msg_0x11._msg_1.content.msg.cmd_type = 0x11;

    float temp_hagl = 0.0f;
    if (plane.ahrs.get_hagl(temp_hagl)) {
        ;
    }
    float temp_asp = 0.0f;
    temp_asp = plane.airspeed.get_airspeed();


    uart_msg_0x11._msg_1.content.msg.gps_lng = ((float)plane.gps.location().lng)/uart_msg_0x11.SF_LNG;
    uart_msg_0x11._msg_1.content.msg.gps_lat = ((float)plane.gps.location().lat)/uart_msg_0x11.SF_LAT;
    uart_msg_0x11._msg_1.content.msg.relative_alt = temp_hagl;
    uart_msg_0x11._msg_1.content.msg.absolute_alt = 0;
    uart_msg_0x11._msg_1.content.msg.baro_alt = 0;
    uart_msg_0x11._msg_1.content.msg.pitch_angle = wrap_180_cd(plane.ahrs.pitch_sensor);
    uart_msg_0x11._msg_1.content.msg.roll_angle = wrap_180_cd(plane.ahrs.roll_sensor);
    uart_msg_0x11._msg_1.content.msg.yaw_angle = wrap_360_cd(plane.ahrs.yaw_sensor);
    uart_msg_0x11._msg_1.content.msg.airspeed = temp_asp;
    uart_msg_0x11._msg_1.content.msg.vel_n = 0;
    uart_msg_0x11._msg_1.content.msg.vel_e = 0;
    uart_msg_0x11._msg_1.content.msg.vel_d = 0;
    uart_msg_0x11._msg_1.content.msg.rest_time = 0;
    uart_msg_0x11._msg_1.content.msg.status = 0;
    uart_msg_0x11._msg_1.content.msg.gps_count = 0;
    uart_msg_0x11._msg_1.content.msg.pos_source = 0;
    uart_msg_0x11._msg_1.content.msg.flight_mode = 0;
    uart_msg_0x11._msg_1.content.msg.time2000 = 0;
    uart_msg_0x11._msg_1.content.msg.vel_lat = 0;
    uart_msg_0x11._msg_1.content.msg.vel_lng = 0;
    uart_msg_0x11._msg_1.content.msg.vel_alt = 0;
    uart_msg_0x11._msg_1.content.msg.pitch_rate = 0;
    uart_msg_0x11._msg_1.content.msg.roll_rate = 0;
    uart_msg_0x11._msg_1.content.msg.yaw_rate = 0;

    uart_msg_0x11.make_sum();
    get_port()->write(uart_msg_0x11._msg_1.content.data, sizeof(uart_msg_0x11._msg_1.content.data));
}

void Uart::send_0x22()
{
    // check send condition
    static uint8_t send_count = 0;
    static bool last_takeoff = false;
    static uint32_t last_ms = millis();

    bool takeoff = plane.is_flying();
    uint32_t now = millis();

    bool status_ok = (!last_takeoff && takeoff);
    bool count_ok = (send_count < 3);
    bool time_ok = (now - last_ms < 200);
    bool need_send = false;

    if (status_ok && count_ok) {
        if (time_ok) {
            send_count++;
            last_ms = now;
            need_send = true;
        }
    } else {
        send_count = 0;
        last_takeoff = takeoff;
    }

    if (!need_send) {return;}
    if (get_port() == nullptr) {return;}
    uart_msg_0x22._msg_1.content.msg.header.head_1 = FD1_msg_reply::PREAMBLE1;
    uart_msg_0x22._msg_1.content.msg.header.head_2 = FD1_msg_reply::PREAMBLE2;
    uart_msg_0x22._msg_1.content.msg.length = uart_msg_0x22._msg_1.length;
    uart_msg_0x22._msg_1.content.msg.cmd_type = 0x22;

    uart_msg_0x22._msg_1.content.msg.status = 0;

    uart_msg_0x22.make_sum();
    get_port()->write(uart_msg_0x22._msg_1.content.data, sizeof(uart_msg_0x22._msg_1.content.data));
}
