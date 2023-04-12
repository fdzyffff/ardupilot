#include "Plane.h"

void Plane::FD1_uart_ts_init() {
    FD1_uart_msg_ts.init();
    FD1_uart_msg_ts.get_msg_ts_in().set_enable();
    FD1_uart_msg_ts.get_msg_ts_out().set_enable();
    FD1_uart_msg_ts.get_msg_ts_route().set_enable();

    FD1_uart_msg_mission.init();
    FD1_uart_msg_mission.get_msg_ts_in().set_enable();
    FD1_uart_msg_mission.get_msg_ts_out().set_enable();
    FD1_uart_msg_mission.get_msg_ts_route().set_enable();
}


void Plane::FD1_uart_ts_update() {

    while (FD1_uart_msg_ts.port_avaliable() > 0) {
        FD1_uart_msg_ts.read();
        FD1_uart_ts_handle_and_route();
        FD1_uart_msg_mission.write();  
    }

    while (FD1_uart_msg_mission.port_avaliable() > 0) {
        FD1_uart_msg_mission.read();
        FD1_uart_mission_handle_and_route();
        FD1_uart_msg_ts.write();  
    }
    if (g2.ts_ahrs_send.get()) {
        FD1_uart_ts_send();
        FD1_uart_msg_ts.write();
    }
}

void Plane::FD1_uart_mission_handle_and_route() {
    if (FD1_uart_msg_mission.get_msg_ts_in()._msg_1.updated) {
        // copy to FD1_uart_msg_ts_route for following uart and mav uses
        memcpy(FD1_uart_msg_ts.get_msg_ts_route()._msg_1.content.data, 
            FD1_uart_msg_mission.get_msg_ts_in()._msg_1.content.data, 
            FD1_uart_msg_mission.get_msg_ts_in()._msg_1.length*sizeof(uint8_t)+3);
        FD1_uart_msg_ts.get_msg_ts_route()._msg_1.length = FD1_uart_msg_mission.get_msg_ts_in()._msg_1.length;
        FD1_uart_msg_ts.get_msg_ts_route()._msg_1.updated = true;
        FD1_uart_msg_ts.get_msg_ts_route()._msg_1.need_send = true;

        // gcs().send_text(MAV_SEVERITY_WARNING, "FD1_uart_msg_mission in %d", FD1_uart_msg_mission.get_msg_ts_in()._msg_1.length);
        // put handle code here
        FD1_uart_msg_mission.get_msg_ts_in()._msg_1.updated = false;
    }
}

void Plane::FD1_uart_ts_handle_and_route() {
    if (FD1_uart_msg_ts.get_msg_ts_in()._msg_1.updated) {
        // copy to FD1_uart_msg_ts_route for following uart and mav uses
        memcpy(FD1_uart_msg_mission.get_msg_ts_route()._msg_1.content.data, 
            FD1_uart_msg_ts.get_msg_ts_in()._msg_1.content.data, 
            FD1_uart_msg_ts.get_msg_ts_in()._msg_1.length*sizeof(uint8_t)+3);
        FD1_uart_msg_mission.get_msg_ts_route()._msg_1.length = FD1_uart_msg_ts.get_msg_ts_in()._msg_1.length;
        FD1_uart_msg_mission.get_msg_ts_route()._msg_1.updated = true;
        FD1_uart_msg_mission.get_msg_ts_route()._msg_1.need_send = true;

        // put handle code here

        // gcs().send_text(MAV_SEVERITY_WARNING, "FD1_uart_msg_ts in %d %d", FD1_uart_msg_ts.get_msg_ts_in()._msg_1.length, FD1_uart_msg_mission.get_msg_ts_route()._msg_1.length);
        // FD1_uart_ts_AHRS_test();
        FD1_uart_msg_ts.get_msg_ts_in()._msg_1.updated = false;
    }
}

void Plane::FD1_uart_ts_send() {
    static uint8_t n_count = 0;

    uint8_t year_out = 0;
    uint8_t month_out = 0;
    uint8_t day_out = 0;
    uint8_t hour_out = 0;
    uint8_t minute_out = 0;
    uint8_t second_out = 0;
    get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out);
    uint32_t second_day = second_out + minute_out*60 + hour_out*60*60;
    second_day *= 100;
    second_day += (gps.time_week_ms() % 1000)/10;

    FD1_msg_ts &tmp_msg = FD1_uart_msg_ts.get_msg_ts_out();

    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ts::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ts::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.head_3 = FD1_msg_ts::PREAMBLE3;
    tmp_msg.set_id(0xB1);

    tmp_msg._msg_1.content.msg.sub_msg.msg_m.type = 0x02;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[0]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[1]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[2]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[3]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[4]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[5]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty[6]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.roll_angle   = (int16_t)((double)(ahrs.roll_sensor)/100.f * tmp_msg.SF_INT16);
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.pitch_angle  = (int16_t)((double)(ahrs.pitch_sensor)/100.f * tmp_msg.SF_INT16);
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.yaw_angle    = (int16_t)((double)(ahrs.yaw_sensor)/100.f * tmp_msg.SF_INT16);
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[1]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[1]=year_out&0b01111111;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[1]+=(month_out&0b00001111)<<7;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[0]=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[0]=(month_out&0b00001111)>>1;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.date[0]+=(day_out&0b00011111)<<3;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.time[2]=second_day%255;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.time[1]=(second_day/255)%255;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.time[0]=(second_day/255)/255;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.gps_heading  = (int16_t)(gps.ground_course_cd()*0.01f * tmp_msg.SF_INT16);
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.empty1=0;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.latitude     = gps.location().lat;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.longitude    = gps.location().lng;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.alt          = gps.location().alt*10;
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.gps_vel_xy   = (int16_t)(gps.ground_speed() * 100.f);
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.gps_hdop     = gps.get_hdop(); // already *100 when read in AP_GPS
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.gps_vdop     = gps.get_vdop(); // already *100 when read in AP_GPS
    tmp_msg._msg_1.content.msg.sub_msg.msg_m.gps_vel_z    = (int16_t)(gps.velocity().z * 100.f);

    tmp_msg.sum_check();
    tmp_msg._msg_1.need_send = true;
    n_count++;
    // if (n_count/20==0) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "%d, %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.latitude, current_loc.lat);
    // }
}


void Plane::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out)
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
}
