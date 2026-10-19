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

uint32_t Uart::get_time_ms(FD1_message::Time_t time)
{
    uint32_t now = time.bits.ms;
    now += time.bits.sec * 1000;
    now += time.bits.min * 60 * 1000;
    now += time.bits.h * 60 * 60 * 1000;
    now += time.bits.day * 24 * 60 * 60 * 1000;
    return now;
}

void Uart::set_tims_ms(FD1_message::Time_t &time)
{
    uint32_t ms_global = cali_last_ms_global + millis() - cali_last_ms;
    uint32_t ms_of_day = ms_global % 86400000UL;
    time.bits.ms = ms_of_day % 1000;
    time.bits.sec = (ms_of_day / 1000) % 60;
    time.bits.min = (ms_of_day / 60000) % 60;
    time.bits.h = (ms_of_day / 3600000) % 24;
    time.bits.day = (ms_global / 86400000UL) & 0x1F;
}

void Uart::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out)
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
