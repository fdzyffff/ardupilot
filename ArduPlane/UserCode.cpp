#include "Plane.h"

void Plane::userhook_init()
{
    uattack.init();
    ufollow.init();
    umission.init();
    udelay.init();
    sim_init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    ufollow.update();
    umission.update();
    udelay.push();
    sim_update();
}

void Plane::userhook_1Hz()
{
    if ((g2.user_print.get() & (1<<0)) && uattack.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uattack.display_info_count_log, uattack.display_info_p1, uattack.display_info_p2, uattack.display_info_p3, uattack.display_info_p4);
        uattack.display_info_new = false;
    }
    if (g2.user_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (g2.user_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate());
    }
    if (g2.user_print.get() & (1<<5)) { // 32
        ufollow.print();
    }

}

void Plane::sim_init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    FD2_uart_msg_ue4.init();
    FD2_uart_msg_ue4.get_msg_ue4_ahrs().set_enable();
    FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal().set_enable();
#endif
}

void Plane::sim_update()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

    FD2_uart_msg_ue4.read();

    SITL::SIM* _sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }
    if (_sitl) {
        const struct SITL::sitl_fdm &fdm = _sitl->state;

        FD2_msg_ue4_ahrs &tmp_msg = FD2_uart_msg_ue4.get_msg_ue4_ahrs();
        tmp_msg._msg_1.content.msg.header.head_1 = FD2_msg_ue4_ahrs::PREAMBLE1;
        tmp_msg._msg_1.content.msg.header.head_2 = FD2_msg_ue4_ahrs::PREAMBLE2;
        tmp_msg._msg_1.content.msg.vehicle_id = 0;
        tmp_msg._msg_1.content.msg.lat = fdm.latitude * 1e7;
        tmp_msg._msg_1.content.msg.lng = fdm.longitude * 1e7;
        tmp_msg._msg_1.content.msg.alt = fdm.altitude*100;
        tmp_msg._msg_1.content.msg.roll = fdm.rollDeg;
        tmp_msg._msg_1.content.msg.pitch = fdm.pitchDeg;
        tmp_msg._msg_1.content.msg.yaw = fdm.yawDeg;
        tmp_msg._msg_1.content.msg.sum_check = 0;

        for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
            tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
        }
        tmp_msg._msg_1.need_send = true;
    }

    FD2_uart_msg_ue4.write();

    FD2_msg2apm_ue4_gimbal &tmp_msg = FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal();
    if (tmp_msg._msg_1.updated) {
        float target_x = degrees(tmp_msg._msg_1.content.msg.yaw);
        float target_y = degrees(tmp_msg._msg_1.content.msg.pitch);
        if (plane.uattack._cam_port_type == 1) {
            plane.uattack._UCam_ptr->handle_info_test(target_x, target_y);
        }
        tmp_msg._msg_1.updated = false;   
    }
#endif
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
