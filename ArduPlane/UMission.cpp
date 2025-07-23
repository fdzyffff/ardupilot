#include "Plane.h"

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
    _uart_control.get_msg_trans().set_enable();
    _uart_control.get_msg_ground().set_enable();
    _uart_link.init();
    _uart_link.get_msg_status().set_enable();
    _uart_link.get_msg_control().set_enable();
    _uart_link.get_msg_trans().set_enable();
    _uart_link.get_msg_ground().set_enable();

    _last_ms = 0;
}

void UMission::update_log()
{
    if (millis() - _last_log_ms < 100) {return;}
    _last_log_ms = millis();
    AP::logger().WriteStreaming("UARL",
                                "TimeUS,valid,type,alt,spd,roll,course",
                                "s------",
                                "F------",
                                "Qffffff",
                                AP_HAL::micros64(),
                                (float)_valid,
                                (float)_control_type,
                                (float)_control_altitude,
                                (float)_control_speed,
                                (float)_control_roll,
                                (float)_control_course);

}


// called at 100 Hz
void UMission::update()
{
    update_uart_read();
    update_uart_send();
    update_valid();
    update_log();
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
    if (_uart_link.initialized() && _uart_link.get_port() != nullptr) {
        while (_uart_link.get_port()->available() > 0) {
            uint8_t temp = _uart_link.get_port()->read();
            _uart_link.read(temp);
            if (_uart_link.get_msg_ground()._msg_1.updated) {
                handle_msg_ground();
            }
        }
    }
}

void UMission::update_uart_send()
{
    // send msg status
    send_status();
    // send msg trans
    send_trans();
}

void UMission::update_valid()
{
    if (_last_ms == 0) {
        _valid = false;
        return;
    }

    if (millis() - _last_ms > 20000) {
        if (_valid) {
            _valid = false;
            gcs().send_text(MAV_SEVERITY_INFO, "[Swarm] Lost control");
        }
    } else {
        if (!_valid) {
            _valid = true;
            gcs().send_text(MAV_SEVERITY_INFO, "[Swarm] Get control");
        }
    }
}

void UMission::handle_msg_control()
{
    FD1_msg_control &tmp_msg = _uart_control.get_msg_control();
    if (tmp_msg._msg_1.updated) {
        _control_type = tmp_msg._msg_1.content.msg.control_mode;
        _control_altitude = tmp_msg._msg_1.content.msg.target_alt_m;
        _control_speed = tmp_msg._msg_1.content.msg.target_airspeed;
        _control_roll = tmp_msg._msg_1.content.msg.target_roll_deg;
        _control_course = tmp_msg._msg_1.content.msg.target_course;
        tmp_msg._msg_1.updated = false;
        _last_ms = millis();
    }

    // gcs().send_text(MAV_SEVERITY_INFO, "_control_type %d",     _control_type);
    // gcs().send_text(MAV_SEVERITY_INFO, "_control_altitude %f", _control_altitude);
    // gcs().send_text(MAV_SEVERITY_INFO, "_control_speed %f",    _control_speed);
    // gcs().send_text(MAV_SEVERITY_INFO, "_control_roll %f",     _control_roll);
    // gcs().send_text(MAV_SEVERITY_INFO, "_control_course %f",   _control_course);
}

void UMission::handle_msg_ground()
{
    // if (_uart_link.get_msg_ground()._msg_1.updated) {
    //     memcpy(_uart_control.get_msg_ground()._msg_1.content.data, 
    //         _uart_link.get_msg_ground()._msg_1.content.data, 
    //         sizeof(_uart_link.get_msg_ground()._msg_1.content.data));
    //     _uart_control.get_msg_ground()._msg_1.need_send = true;
    //     _uart_link.get_msg_ground()._msg_1.updated = false;
    //     _uart_control.write();
    // }
}

void UMission::send_status()
{
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms < 250) {return;}
    _last_send_ms = millis();

    FD1_msg_status &tmp_msg = _uart_link.get_msg_status();
    //uint8_t
    tmp_msg._msg_1.content.msg.number_1 = 0;

    uint8_t tmp_sysid = 0;
    if (plane.g.sysid_this_mav <= 15) {
        tmp_sysid = 0x0A + (0x10 * plane.g.sysid_this_mav);
    }
    //uint8_t
    tmp_msg._msg_1.content.msg.number_2 = tmp_sysid;
    //uint16_t
    tmp_msg._msg_1.content.msg.length = 128;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_1 = 0xAA;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_2 = 0xAA;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_3 = 0x00;
    //uint8_t
    tmp_msg._msg_1.content.msg.type = 0xDD;
    //uint16_t
    tmp_msg._msg_1.content.msg.data_length = 128;
    //uint32_t
    tmp_msg._msg_1.content.msg.system_time_s = AP_HAL::millis()/1000;

    uint8_t year_out = 0;
    uint8_t month_out = 0;
    uint8_t day_out = 0;
    uint8_t hour_out = 0;
    uint8_t minute_out = 0;
    uint8_t second_out = 0;
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out);
    }

    //uint16_t
    tmp_msg._msg_1.content.msg.utc_year = (uint16_t)(year_out)+2000;
    //uint8_t
    tmp_msg._msg_1.content.msg.utc_month = month_out;
    //uint8_t
    tmp_msg._msg_1.content.msg.utc_day = day_out;
    //uint8_t
    tmp_msg._msg_1.content.msg.utc_hour = hour_out;
    //uint8_t
    tmp_msg._msg_1.content.msg.utc_minute = minute_out;
    //uint8_t
    tmp_msg._msg_1.content.msg.utc_second = second_out;
    //uint8_t
    tmp_msg._msg_1.content.msg.gps_count = AP::gps().num_sats();
    //uint8_t
    tmp_msg._msg_1.content.msg.bd_count = AP::gps().num_sats();
    //uint8_t
    tmp_msg._msg_1.content.msg.gps_ok = (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D);

    // double tmp_lng_deg = 0.0f;
    // if ((AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D)) {
    //     tmp_lng_deg = ((double)plane.current_loc.lng)*1e-7;
    // }
    // //double
    // tmp_msg._msg_1.content.msg.lng_deg = tmp_lng_deg;

    // double tmp_lat_deg = 0.0f;
    // if ((AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D)) {
    //     tmp_lat_deg = ((double)plane.current_loc.lat)*1e-7;
    // }
    // //double
    // tmp_msg._msg_1.content.msg.lat_deg = tmp_lat_deg;
    //float
    tmp_msg._msg_1.content.msg.alt_m = plane.relative_ground_altitude(false);

    Vector3f pos;
    if (AP::ahrs().get_relative_position_NED_origin(pos)) {
        ;
    }
    //float
    tmp_msg._msg_1.content.msg.pos_n_m = pos.x;
    //float
    tmp_msg._msg_1.content.msg.pos_e_m = pos.y;
    //float
    tmp_msg._msg_1.content.msg.pos_d_m = pos.z;

    Vector3f vel;
    if (AP::ahrs().get_velocity_NED(vel)) {
        ;
    }
    //float
    tmp_msg._msg_1.content.msg.vel_n_ms = vel.x;
    //float
    tmp_msg._msg_1.content.msg.vel_e_ms = vel.y;
    //float
    tmp_msg._msg_1.content.msg.vel_d_ms = vel.z;
    //float
    tmp_msg._msg_1.content.msg.roll_deg = degrees(AP::ahrs().get_roll());
    //float
    tmp_msg._msg_1.content.msg.pitch_deg = degrees(AP::ahrs().get_pitch());
    //float
    tmp_msg._msg_1.content.msg.yaw_deg = degrees(AP::ahrs().get_yaw());

    const Vector3f &omega = AP::ahrs().get_gyro();
    //float
    tmp_msg._msg_1.content.msg.roll_rate_degs = degrees(omega.x);
    //float
    tmp_msg._msg_1.content.msg.pitch_rate_degs = degrees(omega.y);
    //float
    tmp_msg._msg_1.content.msg.yaw_rate_degs = degrees(omega.z);

    const Vector3f &acc = AP::ahrs().get_accel();
    //float
    tmp_msg._msg_1.content.msg.acc_x_g = acc.x/GRAVITY_MSS;
    //float
    tmp_msg._msg_1.content.msg.acc_y_g = acc.y/GRAVITY_MSS;
    //float
    tmp_msg._msg_1.content.msg.acc_z_g = acc.z/GRAVITY_MSS;
    //float
    tmp_msg._msg_1.content.msg.airspeed = plane.airspeed.get_airspeed();
    //float
    tmp_msg._msg_1.content.msg.groundspeed = plane.gps.ground_speed();
    //float
    tmp_msg._msg_1.content.msg.baro_alt_m = plane.barometer.get_altitude();
    //float
    tmp_msg._msg_1.content.msg.batt_volt = 0;
    //float
    tmp_msg._msg_1.content.msg.batt_current = 0;
    //uint16_t
    tmp_msg._msg_1.content.msg.motor_rpm = 0;
    //uint8_t
    tmp_msg._msg_1.content.msg.throttle = 0;

    uint8_t tmp_control_mode = 2;
    if (plane.control_mode == &plane.mode_mission) {
        tmp_control_mode = 3;
    }
    //uint8_t
    tmp_msg._msg_1.content.msg.control_mode = tmp_control_mode;
    //uint8_t
    tmp_msg._msg_1.content.msg.flight_stage = 0;
    //uint8_t
    tmp_msg._msg_1.content.msg.parachute = 0;
    //uint8_t
    tmp_msg._msg_1.content.msg.radio = 1;

    tmp_msg.make_sum();

    tmp_msg._msg_1.need_send = true;

    memcpy(_uart_control.get_msg_status()._msg_1.content.data, tmp_msg._msg_1.content.data, sizeof(tmp_msg._msg_1.content.data));
    _uart_control.get_msg_status()._msg_1.need_send = true;

    _uart_link.write();
    _uart_control.write();
}

void UMission::send_trans()
{
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms < 250) {return;}
    _last_send_ms = millis();

    FD1_msg_trans &tmp_msg = _uart_link.get_msg_trans();
    //uint8_t
    tmp_msg._msg_1.content.msg.number_1 = 0;

    uint8_t tmp_sysid = 0;
    if (plane.g.sysid_this_mav <= 15) {
        tmp_sysid = 0x0A + (0x10 * plane.g.sysid_this_mav);
    }
    //uint8_t
    tmp_msg._msg_1.content.msg.number_2 = tmp_sysid;
    //uint16_t
    tmp_msg._msg_1.content.msg.length = 49;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_1 = 0xAA;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_2 = 0xAA;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_3 = 0x00;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_4 = 0xDD;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_5 = 0x00;
    //uint8_t
    tmp_msg._msg_1.content.msg.head_6 = 0x00;
    //uint32_t
    tmp_msg._msg_1.content.msg.system_time_s = AP_HAL::millis()/1000;

    Vector3f pos;
    if (AP::ahrs().get_relative_position_NED_origin(pos)) {
        ;
    }
    //float
    tmp_msg._msg_1.content.msg.pos_n_m = pos.x;
    //float
    tmp_msg._msg_1.content.msg.pos_e_m = pos.y;
    //float
    tmp_msg._msg_1.content.msg.pos_d_m = pos.z;

    Vector3f vel;
    if (AP::ahrs().get_velocity_NED(vel)) {
        ;
    }
    //float
    tmp_msg._msg_1.content.msg.vel_n_ms = vel.x;
    //float
    tmp_msg._msg_1.content.msg.vel_e_ms = vel.y;
    //float
    tmp_msg._msg_1.content.msg.vel_d_ms = vel.z;
    //float
    tmp_msg._msg_1.content.msg.roll_deg = degrees(AP::ahrs().get_roll());
    //float
    tmp_msg._msg_1.content.msg.pitch_deg = degrees(AP::ahrs().get_pitch());
    //float
    tmp_msg._msg_1.content.msg.yaw_deg = degrees(AP::ahrs().get_yaw());
    //float
    tmp_msg._msg_1.content.msg.airspeed = plane.airspeed.get_airspeed();
    //uint8_t
    tmp_msg._msg_1.content.msg.throttle = 0;

    uint8_t tmp_control_mode = 2;
    if (plane.control_mode == &plane.mode_mission) {
        tmp_control_mode = 3;
    }
    //uint8_t
    tmp_msg._msg_1.content.msg.control_mode = tmp_control_mode;
    //uint8_t
    tmp_msg._msg_1.content.msg.flight_stage = 0;
    //uint8_t
    tmp_msg._msg_1.content.msg.parachute = 0;
    //uint8_t
    tmp_msg._msg_1.content.msg.radio = 1;

    tmp_msg.make_sum();

    tmp_msg._msg_1.need_send = true;

    _uart_link.write();
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
                    _control_type = (int16_t)(packet.param1) == 1;
                    _control_altitude = packet.param2;
                    _control_speed = packet.param3;
                    _control_roll = packet.param4;
                    _control_course = packet.param5;
                }
                break;
            default:
                break;
        }
    }
}

void UMission::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out)
{
    uint32_t days;
    uint32_t year;
    uint32_t month;
    uint32_t date;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
    uint32_t weekday;

    static uint32_t old_year;
    static uint32_t old_month;
    static uint32_t old_date;
    static uint32_t old_week = -1;

    uint32_t days_of_month[13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };

    uint16_t GPS_week = AP::gps().time_week();
    uint32_t time_of_week_s = AP::gps().time_week_ms() / 1000;

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
    month_out = month;
    day_out = date;
    hour_out = hour;
    minute_out = minute;
    second_out = second;
}
