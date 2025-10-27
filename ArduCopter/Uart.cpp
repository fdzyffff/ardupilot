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
    static uint8_t char_end = 0;
    static uint8_t hex_temp = 0;
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();

        // uart_msg_0919_p1.parse(temp);
        // handle_0919_p1();

        // uart_msg_0919_p2.parse(temp);
        // handle_0919_p2();

        // gcs().send_text(MAV_SEVERITY_INFO, "temp in %x", temp);
        if (temp == 44) { //","
            char_end = 0;
        } else if (char_end == 0) {
            hex_temp = 0;
            char_end++;
            if ((temp >= 48) && (temp <=57)) {
                hex_temp += (temp - 48) * 16;
            }
            if ((temp >= 65) && (temp <=70)) {
                hex_temp += (temp - 55) * 16;
            }
            if ((temp >= 97) && (temp <=102)) {
                hex_temp += (temp - 87) * 16;
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "t 1 %x", temp);
        } else if (char_end == 1) {
            char_end = 0;
            if ((temp >= 48) && (temp <=57)) {
                hex_temp += (temp - 48);
            }
            if ((temp >= 65) && (temp <=70)) {
                hex_temp += (temp - 55);
            }
            if ((temp >= 97) && (temp <=102)) {
                hex_temp += (temp - 87);
            }

            uart_msg_0919_p1.parse(hex_temp);
            handle_0919_p1();

            uart_msg_0919_p2.parse(hex_temp);
            handle_0919_p2();

            // gcs().send_text(MAV_SEVERITY_INFO, "t 2 %x", temp);
        }
    }
}

void Uart::write_uart()
{
    send_0919_p5();
}

void Uart::handle_0919_p1()
{
    if (uart_msg_0919_p1._msg_1.updated) {
        gcs().send_text(MAV_SEVERITY_INFO, "-> New Target WPs [%d]<-", uart_msg_0919_p1._msg_1.content.msg.wp_number);
        if (uart_msg_0919_p1._msg_1.content.msg.wp_number > 0) {
            if (copter.flightmode->mode_number() == Mode::Number::MISSION) {
                for (uint8_t i_wp = 0; i_wp < uart_msg_0919_p1._msg_1.content.msg.wp_number; i_wp++) {
                    Location temp_loc;
                    unpack_0919_lng(temp_loc.lng, uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].content_wp.wp_lng);
                    unpack_0919_lat(temp_loc.lat, uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].content_wp.wp_lat);
                    unpack_0919_alt(temp_loc.alt, uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].content_wp.wp_alt);
                    uint8_t temp_spd = uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].content_wp.wp_spd;

                    if (copter.g2.user_parameters._wp_alt.get() > 0) {
                        temp_loc.set_alt_cm(copter.g2.user_parameters._wp_alt.get(), Location::AltFrame::ABOVE_HOME);
                    }
                    copter.mode_mission.set_loc(temp_loc, temp_spd, i_wp);
                }
                copter.mode_mission.set_wp_number(uart_msg_0919_p1._msg_1.content.msg.wp_number);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "-> Err, Not in MISSION Mode <-");
            }
        }
        // pack ack report p3
        send_0919_p3();

        uart_msg_0919_p1._msg_1.updated = false;
    }
}

void Uart::handle_0919_p2()
{
    if (uart_msg_0919_p2._msg_1.updated) {
        gcs().send_text(MAV_SEVERITY_INFO, "-> New CMD [%d]<-", uart_msg_0919_p2._msg_1.content.msg.control_type);
        if (copter.flightmode->mode_number() == Mode::Number::MISSION) {
            switch (uart_msg_0919_p2._msg_1.content.msg.control_type) {
                case 0:{
                    copter.mode_mission.set_state(ModeMission::MISSION_State::LAND);
                }
                break;
                case 1:{
                    copter.mode_mission.set_state(ModeMission::MISSION_State::Takeoff);
                }
                break;
                case 2:{
                    copter.mode_mission.set_state(ModeMission::MISSION_State::RETURN);
                }
                break;
                default:
                break;
            }
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "-> Err, Not in MISSION Mode <-");
        }
        // pack ack report p4
        send_0919_p4();

        uart_msg_0919_p2._msg_1.updated =false;
    }
}

void Uart::send_0919_p3()
{
    // check send condition
    if (get_port() == nullptr) {return;}
    uart_msg_0919_p3._msg_1.content.msg.T_Type = uart_msg_0919_p1._msg_1.content.msg.T_Type;
    uart_msg_0919_p3._msg_1.content.msg.T_Subtype = uart_msg_0919_p1._msg_1.content.msg.T_Subtype;
    uart_msg_0919_p3._msg_1.content.msg.RID = uart_msg_0919_p1._msg_1.content.msg.RID;
    uart_msg_0919_p3._msg_1.content.msg.PID = uart_msg_0919_p1._msg_1.content.msg.PID;
    uart_msg_0919_p3._msg_1.content.msg.C_Idegree = 0;
    if (copter.position_ok()) {
        pack_0919_lng(copter.current_loc.lng, uart_msg_0919_p3._msg_1.content.msg.loc.content_wp.wp_lng);
        pack_0919_lng(copter.current_loc.lat, uart_msg_0919_p3._msg_1.content.msg.loc.content_wp.wp_lat);
        int32_t alt_cm;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            pack_0919_alt(alt_cm, uart_msg_0919_p3._msg_1.content.msg.loc.content_wp.wp_alt);
        }
    }

    uint8_t year_out;
    uint8_t month_out;
    uint8_t day_out;
    uint8_t hour_out;
    uint8_t minute_out;
    uint8_t second_out;
    uint16_t second_ms_out;
    get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out, second_ms_out);

    uart_msg_0919_p3._msg_1.content.msg.start_time.year = year_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.month = month_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.day = day_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.hour = hour_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.minute = minute_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.second = second_out;
    uart_msg_0919_p3._msg_1.content.msg.start_time.second_ms = second_ms_out;

    uart_msg_0919_p3._msg_1.content.msg.T_Type_c = uart_msg_0919_p1._msg_1.content.msg.T_Type_c;
    uart_msg_0919_p3._msg_1.content.msg.T_Subtype_c = uart_msg_0919_p1._msg_1.content.msg.T_Subtype_c;
    uart_msg_0919_p3._msg_1.content.msg.RID_c = uart_msg_0919_p1._msg_1.content.msg.RID_c;
    uart_msg_0919_p3._msg_1.content.msg.PID_c = uart_msg_0919_p1._msg_1.content.msg.PID_c;
    uart_msg_0919_p3._msg_1.content.msg.wp_number = uart_msg_0919_p1._msg_1.content.msg.wp_number;
    for (uint8_t i_wp = 0; i_wp < uart_msg_0919_p1._msg_1.content.msg.wp_number; i_wp++) {
        memcpy(uart_msg_0919_p3._msg_1.content.msg.wp_data[i_wp].data, uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].data, sizeof(uart_msg_0919_p1._msg_1.content.msg.wp_data[i_wp].data));
    }
    uart_msg_0919_p3._msg_1.length = 37 + 1 + 16 * uart_msg_0919_p1._msg_1.content.msg.wp_number;

    uart_msg_0919_p3.make_sum();
    get_port()->write(uart_msg_0919_p3._msg_1.content.data, uart_msg_0919_p3._msg_1.length+6);
}

void Uart::send_0919_p4()
{
    // check send condition
    if (get_port() == nullptr) {return;}
    uart_msg_0919_p4._msg_1.content.msg.T_Type = uart_msg_0919_p2._msg_1.content.msg.T_Type;
    uart_msg_0919_p4._msg_1.content.msg.T_Subtype = uart_msg_0919_p2._msg_1.content.msg.T_Subtype;
    uart_msg_0919_p4._msg_1.content.msg.RID = uart_msg_0919_p2._msg_1.content.msg.RID;
    uart_msg_0919_p4._msg_1.content.msg.PID = uart_msg_0919_p2._msg_1.content.msg.PID;
    uart_msg_0919_p4._msg_1.content.msg.C_Idegree = 0;
    if (copter.position_ok()) {
        pack_0919_lng(copter.current_loc.lng, uart_msg_0919_p4._msg_1.content.msg.loc.content_wp.wp_lng);
        pack_0919_lng(copter.current_loc.lat, uart_msg_0919_p4._msg_1.content.msg.loc.content_wp.wp_lat);
        int32_t alt_cm;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            pack_0919_lng(alt_cm, uart_msg_0919_p4._msg_1.content.msg.loc.content_wp.wp_alt);
        }
    }

    uint8_t year_out;
    uint8_t month_out;
    uint8_t day_out;
    uint8_t hour_out;
    uint8_t minute_out;
    uint8_t second_out;
    uint16_t second_ms_out;
    get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out, second_ms_out);

    uart_msg_0919_p4._msg_1.content.msg.start_time.year = year_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.month = month_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.day = day_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.hour = hour_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.minute = minute_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.second = second_out;
    uart_msg_0919_p4._msg_1.content.msg.start_time.second_ms = second_ms_out;

    uart_msg_0919_p4._msg_1.content.msg.T_Type_c = uart_msg_0919_p2._msg_1.content.msg.T_Type_c;
    uart_msg_0919_p4._msg_1.content.msg.T_Subtype_c = uart_msg_0919_p2._msg_1.content.msg.T_Subtype_c;
    uart_msg_0919_p4._msg_1.content.msg.RID_c = uart_msg_0919_p2._msg_1.content.msg.RID_c;
    uart_msg_0919_p4._msg_1.content.msg.PID_c = uart_msg_0919_p2._msg_1.content.msg.PID_c;
    uart_msg_0919_p4._msg_1.content.msg.control_type = uart_msg_0919_p2._msg_1.content.msg.control_type;
    uart_msg_0919_p4._msg_1.content.msg.tof_alt = uart_msg_0919_p2._msg_1.content.msg.tof_alt;
    uart_msg_0919_p4._msg_1.length = 39;

    uart_msg_0919_p4.make_sum();
    get_port()->write(uart_msg_0919_p4._msg_1.content.data, uart_msg_0919_p4._msg_1.length+6);
}

void Uart::send_0919_p5()
{
    // check send condition
    if (get_port() == nullptr) {return;}
    static uint32_t _last_p5_ms = millis();
    if (millis() - _last_p5_ms < 5000) {
        return;
    }
    _last_p5_ms = millis();
    uart_msg_0919_p5.make_sum();
    get_port()->write(uart_msg_0919_p5._msg_1.content.data, uart_msg_0919_p5._msg_1.length);
}

void Uart::unpack_0919_lng(int32_t& lng_out, uint8_t lng_in[6]) {
    double lng_p1 = 0.0;
    double lng_p2 = 0.0;
    uint32_t lng_bai   = (lng_in[0]) & 0b00001111;
    uint32_t lng_shi   = (lng_in[1]>>4) & 0b00001111;
    uint32_t lng_ge    = (lng_in[1]) & 0b00001111;
    uint32_t lng_fshi  = (lng_in[2]>>4) & 0b00001111;
    uint32_t lng_fge   = (lng_in[2]) & 0b00001111;
    uint32_t lng_fshif = (lng_in[3]>>4) & 0b00001111;
    uint32_t lng_fbaif = (lng_in[3]) & 0b00001111;
    uint32_t lng_fqf   = (lng_in[4]>>4) & 0b00001111;
    uint32_t lng_fwf   = (lng_in[4]) & 0b00001111;
    uint32_t lng_fswf  = (lng_in[5]>>4) & 0b00001111;
    uint32_t lng_fbwf  = (lng_in[5]) & 0b00001111;
    lng_p1 += ((float)lng_bai)   * 100.f;
    lng_p1 += ((float)lng_shi)   * 10.f;
    lng_p1 += ((float)lng_ge)    * 1.f;
    lng_p2 += ((float)lng_fshi)  * 10.f;
    lng_p2 += ((float)lng_fge)   * 1.f;
    lng_p2 += ((float)lng_fshif) * 0.1f;
    lng_p2 += ((float)lng_fbaif) * 0.01f;
    lng_p2 += ((float)lng_fqf)   * 0.001f;
    lng_p2 += ((float)lng_fwf)   * 0.0001f;
    lng_p2 += ((float)lng_fswf)  * 0.00001f;
    lng_p2 += ((float)lng_fbwf)  * 0.000001f;
    double lng_double = lng_p1 + (lng_p2/60.f);
    lng_out = (int32_t)(lng_double*1e7f);
}

void Uart::unpack_0919_lat(int32_t& lat_out, uint8_t lat_in[5]) {
    double lat_p1 = 0.0;
    double lat_p2 = 0.0;
    uint32_t lat_shi   = (lat_in[0]>>4) & 0b00001111;
    uint32_t lat_ge    = (lat_in[0]) & 0b00001111;
    uint32_t lat_fshi  = (lat_in[1]>>4) & 0b00001111;
    uint32_t lat_fge   = (lat_in[1]) & 0b00001111;
    uint32_t lat_fshif = (lat_in[2]>>4) & 0b00001111;
    uint32_t lat_fbaif = (lat_in[2]) & 0b00001111;
    uint32_t lat_fqf   = (lat_in[3]>>4) & 0b00001111;
    uint32_t lat_fwf   = (lat_in[3]) & 0b00001111;
    uint32_t lat_fswf  = (lat_in[4]>>4) & 0b00001111;
    uint32_t lat_fbwf  = (lat_in[4]) & 0b00001111;
    lat_p1 += ((float)lat_shi)   * 10.f;
    lat_p1 += ((float)lat_ge)    * 1.f;
    lat_p2 += ((float)lat_fshi)  * 10.f;
    lat_p2 += ((float)lat_fge)   * 1.f;
    lat_p2 += ((float)lat_fshif) * 0.1f;
    lat_p2 += ((float)lat_fbaif) * 0.01f;
    lat_p2 += ((float)lat_fqf)   * 0.001f;
    lat_p2 += ((float)lat_fwf)   * 0.0001f;
    lat_p2 += ((float)lat_fswf)  * 0.00001f;
    lat_p2 += ((float)lat_fbwf)  * 0.000001f;
    double lat_double = lat_p1 + (lat_p2/60.f);
    lat_out = (int32_t)(lat_double*1e7f);
}

void Uart::unpack_0919_alt(int32_t& alt_out, uint8_t alt_in[4]) {
    float alt_float = 0.0;
    uint32_t alt_sign  = (alt_in[0]>>7) & 0b00000001;
    uint32_t alt_qian  = (alt_in[0]) & 0b00001111;
    uint32_t alt_bai   = (alt_in[1]>>4) & 0b00001111;
    uint32_t alt_shi   = (alt_in[1]) & 0b00001111;
    uint32_t alt_ge    = (alt_in[2]>>4) & 0b00001111;
    uint32_t alt_fshif = (alt_in[2]) & 0b00001111;
    uint32_t alt_fbaif = (alt_in[3]>>4) & 0b00001111;
    uint32_t alt_fqf   = (alt_in[3]) & 0b00001111;
    alt_float += ((float)alt_qian)  * 1000.f;
    alt_float += ((float)alt_bai)   * 100.f;
    alt_float += ((float)alt_shi)   * 10.f;
    alt_float += ((float)alt_ge)    * 1.f;
    alt_float += ((float)alt_fshif) * 0.1f;
    alt_float += ((float)alt_fbaif) * 0.01f;
    alt_float += ((float)alt_fqf)   * 0.001f;
    if (alt_sign == 1) {
        alt_float *= -1.0f;
    }
    alt_out = (int32_t)(alt_float*1e2f);
}

void Uart::pack_0919_lng(int32_t& lng_in, uint8_t lng_out[6]) {
    ;
}

void Uart::pack_0919_lat(int32_t& lat_in, uint8_t lat_out[5]) {
    ;
}

void Uart::pack_0919_alt(int32_t& alt_in, uint8_t alt_out[4]) {
    ;
}

void Uart::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out, uint16_t &second_ms_out)
{
    uint32_t days;
    uint32_t weekday;

    uint32_t year;
    uint32_t month;
    uint32_t date;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
    uint32_t second_ms;

    static uint32_t old_year;
    static uint32_t old_month;
    static uint32_t old_date;
    static uint32_t old_week = -1;

    uint32_t days_of_month[13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };

    uint16_t GPS_week = AP::gps().time_week();
    uint32_t time_of_week_s = AP::gps().time_week_ms() / 1000;

    second_ms = AP::gps().time_week_ms() % 1000;


    time_of_week_s += 0;  // hour shift*60*60 time zone, now use utc
    if (time_of_week_s >= 604800)  // 7 * 24 * 60 *60 = 604800
    {
        time_of_week_s -= 604800;
        GPS_week++;
    }

    if (GPS_week != old_week)
    {
        year = 1980;
        month = 1;
        date = 6;

        days = GPS_week * 7;

        while (1)
        {
            if (year % 4 == 0)
            {
                if (days >= 366)
                {
                    days -= 366;
                    year++;
                }
                else
                    break;
            }
            else
            {
                if (days >= 365)
                {
                    days -= 365;
                    year++;
                }
                else
                    break;
            }
        }

        while (1)
        {
            if (month == 2 && (year % 4 == 0))
                if (days >= (days_of_month[month] + 1))
                {
                    days -= days_of_month[month] + 1;
                    month++;
                }
                else
                    break;
            else
                if (days >= (days_of_month[month]))
                {
                    days -= days_of_month[month];
                    month++;
                }
                else
                    break;
        }

        date += days;

        old_year = year;
        old_month = month;
        old_date = date;
        old_week = GPS_week;
    }
    else
    {
        year = old_year;
        month = old_month;
        date = old_date;
    }

    hour = 0;
    minute = 0;
    second = 0;
    weekday = 0;

    while (1)
    {
        if (time_of_week_s >= 24 * 3600)
        {
            time_of_week_s -= 24 * 3600;
            date++;
            weekday++;
            if (date> (((year % 4 == 0) && month == 2) ? days_of_month[month] + 1 : days_of_month[month]))
            {
                date = 1;
                month++;
                if (month>12)
                {
                    month = 1;
                    year++;
                }
            }
        }
        else
            break;
    }

    while (1)
    {
        if (time_of_week_s >= 3600)
        {
            time_of_week_s -= 3600;
            hour++;
        }
        else
            break;
    }

    while (1)
    {
        if (time_of_week_s >= 60)
        {
            time_of_week_s -= 60;
            minute++;
        }
        else
            break;
    }

    second += time_of_week_s;

    year_out = (uint8_t)(year - 2000);
    month_out = (uint8_t)month;
    day_out = (uint8_t)date;
    hour_out = (uint8_t)hour;
    minute_out = (uint8_t)minute;
    second_out = (uint8_t)second;
    second_ms_out = (uint16_t)second_ms;
}

void Uart::handle_msg(const mavlink_message_t &msg)
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
                    uart_msg_0919_p2._msg_1.content.msg.control_type = (uint8_t)packet.param1;
                    uart_msg_0919_p2._msg_1.updated = true;
                    gcs().send_text(MAV_SEVERITY_INFO, "MIS Test CMD [%d]", uart_msg_0919_p2._msg_1.content.msg.control_type);
                    handle_0919_p2();
                }
                break;
            default:
                break;
        }
    }
}
