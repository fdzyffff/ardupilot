#include "Plane.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UCam_DYT::UCam_DYT(UAttack &frotend_in, AP_HAL::UARTDriver* port_in):
    UCam_base(frotend_in)
{
    FD1_uart_ptr = new FD1_UART(port_in);
    FD1_uart_ptr->get_msg_DYTTELEM().set_enable();
    FD1_uart_ptr->get_msg_DYTTARGET().set_enable();
    FD1_uart_ptr->get_msg_APM2DYTCONTROL().set_enable();
    FD1_uart_ptr->get_msg_APM2DYTTELEM().set_enable();
    _yaw_rate_filter.set_cutoff_frequency(100.f, 20.f);
    return;
}

void UCam_DYT::update() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    _yaw_rate_filter.apply(degrees(plane.ahrs.get_yaw_rate_earth()));

    FD1_uart_ptr->read();
    FD1_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    if (tmp_msg._msg_1.updated) {
        _valid = true;
        _last_ms = millis();
        // DYT -> APM
        handle_info();
        tmp_msg._msg_1.updated = false;
    }

    if (tnow - _last_ms > 1000) {
        _valid = false;
        _pitch_filter.reset();
        _yaw_filter.reset();
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }

    fill_state_msg(); // APM -> DYT
    foward_DYT_mavlink(); // DYT -> APM -> GCS
}

// APM -> DYT
void UCam_DYT::fill_state_msg()
{
    if (plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }
    static uint32_t _last_send_ms = 0;
    uint32_t tnow = millis();
    if (tnow - _last_send_ms < 20) {
        return;
    }

    uint8_t year_out = 0;
    uint8_t month_out = 0;
    uint8_t day_out = 0;
    uint8_t hour_out = 0;
    uint8_t minute_out = 0;
    uint8_t second_out = 0;
    uint8_t second_10ms_out = 0;
    plane.get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out, second_10ms_out);

    FD1_msg_APM2DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_APM2DYTTELEM();

    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_APM2DYTTELEM::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_APM2DYTTELEM::PREAMBLE2;

    float temp_hagl = 0.0f;;
    if (plane.ahrs.get_hagl(temp_hagl)) {
        ;
    }
    float temp_asp = 0.0f;
    temp_asp = plane.airspeed.get_airspeed();
    tmp_msg._msg_1.content.msg.roll = (int16_t)(wrap_180_cd(plane.ahrs.pitch_sensor));
    tmp_msg._msg_1.content.msg.pitch = (int16_t)(wrap_180_cd(plane.ahrs.roll_sensor));
    tmp_msg._msg_1.content.msg.yaw = (int16_t)(wrap_180_cd(plane.ahrs.yaw_sensor));
    tmp_msg._msg_1.content.msg.lat = plane.gps.location().lat;
    tmp_msg._msg_1.content.msg.lng = plane.gps.location().lng;
    tmp_msg._msg_1.content.msg.alt_abs = (int16_t)(plane.gps.location().alt*20);
    tmp_msg._msg_1.content.msg.alt_rel = (int16_t)(temp_hagl*5.0f);
    tmp_msg._msg_1.content.msg.year = year_out;
    tmp_msg._msg_1.content.msg.month = month_out;
    tmp_msg._msg_1.content.msg.day = day_out;
    tmp_msg._msg_1.content.msg.hour = hour_out;
    tmp_msg._msg_1.content.msg.minute = minute_out;
    tmp_msg._msg_1.content.msg.second = second_out;
    tmp_msg._msg_1.content.msg.second_10ms = second_10ms_out;
    tmp_msg._msg_1.content.msg.airspeed = (uint16_t)temp_asp;
    tmp_msg._msg_1.content.msg.gpsspeed = (uint16_t)(plane.gps.ground_speed() * 100.f);

    tmp_msg.sum_check();
    tmp_msg._msg_1.need_send = true;

    FD1_uart_ptr->write();

    _last_send_ms = tnow;
}


void UCam_DYT::do_cmd() {
    ;
}

bool UCam_DYT::is_valid() {
    return _valid;
}

void UCam_DYT::handle_info() {
    FD1_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    float p1 = (float)(tmp_msg._msg_1.content.msg.target_x) / 0.005f; // x-axis
    float p2 = (float)(tmp_msg._msg_1.content.msg.target_y) / 0.005f; // y-axis

    _frotend.bf_info.x = p1; // degree
    _frotend.bf_info.y = p2; // degree

    Matrix3f tmp_m;
    tmp_m.from_euler(plane.ahrs.roll, plane.ahrs.pitch, 0.0f);

    float dist_z = -tanf(radians(p1));
    float dist_y = tanf(radians(p2));

    Vector3f tmp_input = Vector3f(1.0f,dist_y,dist_z);
    Vector3f tmp_output = tmp_m*tmp_input;

    float angle_pitch = wrap_180(degrees(atan2f(-tmp_output.z, tmp_output.x)));
    float angle_yaw = wrap_180(degrees(atan2f(tmp_output.y, tmp_output.x)));

    _frotend.ef_info.x = wrap_360(angle_yaw + degrees(plane.ahrs.yaw));
    _frotend.ef_info.y = angle_pitch;

    _yaw_filter.update(angle_yaw, millis());
    _pitch_filter.update(angle_pitch, millis());

    _frotend.ef_rate_info.x = _yaw_rate_filter.get() + _yaw_filter.slope()*1000.f;
    _frotend.ef_rate_info.y = _pitch_filter.slope()*1000.f;

    _frotend.udpate_control_value();

    _frotend.display_info_p1 = _frotend.bf_info.x;
    _frotend.display_info_p2 = _frotend.bf_info.y;
    _frotend.display_info_p3 = _frotend.ef_info.x;
    _frotend.display_info_p4 = _frotend.ef_info.y;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
}

// GCS -> APM -> DYT
void UCam_DYT::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_MY_OPTIC_DATA) {
        // decode packet
        mavlink_my_optic_data_t my_optic_data;
        mavlink_msg_my_optic_data_decode(&msg, &my_optic_data);
        uint8_t i = 0;
        for (i = 0; i < 32; i++) {
            FD1_uart_ptr->read(my_optic_data.data[i]);
        }
    }
    if (FD1_uart_ptr->get_msg_APM2DYTCONTROL()._msg_1.updated) {
        FD1_uart_ptr->get_msg_APM2DYTCONTROL()._msg_1.need_send = true;
        FD1_uart_ptr->get_msg_APM2DYTCONTROL()._msg_1.updated = false;
    }
    FD1_uart_ptr->write();
}

// DYT -> APM -> GCS
void UCam_DYT::foward_DYT_mavlink()
{
    static uint32_t _last_send_ms = 0;
    static uint8_t _order = 0;
    uint32_t tnow = millis();
    // uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    if (tnow - _last_send_ms > 500) {
        _last_send_ms = tnow;
        _order++;
        if (_order % 2 == 0) {
            _order = 1;
        }
        if (_order % 2 == 0) {
            for (uint8_t i=0; i<gcs().num_gcs(); i++) {
                mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 50) {
                    mavlink_my_optic_data_t my_optic_data;
                    memcpy(my_optic_data.data, FD1_uart_ptr->get_msg_DYTTARGET()._msg_1.content.data, sizeof(FD1_uart_ptr->get_msg_DYTTARGET()._msg_1.content.data));
                    mavlink_msg_my_optic_data_send(
                        channel,
                        0,
                        my_optic_data.data);
                }
            }
        } else {
            for (uint8_t i=0; i<gcs().num_gcs(); i++) {
                mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 50) {
                    mavlink_my_optic_data_t my_optic_data;
                    memcpy(my_optic_data.data, FD1_uart_ptr->get_msg_DYTTELEM()._msg_1.content.data, sizeof(FD1_uart_ptr->get_msg_DYTTELEM()._msg_1.content.data));
                    mavlink_msg_my_optic_data_send(
                        channel,
                        0,
                        my_optic_data.data);
                }
            }
        }
    }
}

void Plane::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out, uint8_t &second_10ms_out)
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

    uint16_t GPS_week = gps.time_week();
    uint32_t time_of_week_s = gps.time_week_ms() / 1000;

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
    second_10ms_out = (gps.time_week_ms() % 1000) / 10;;
}
