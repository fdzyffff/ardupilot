/*
   Uart: ArduPlane 业务层串口接口
   封装 HXKY_Uart 底层库，处理业务逻辑
*/

#include "Plane.h"

Uart::Uart()
{
}

void Uart::init()
{
    AP::hxky_uart().init();
}

void Uart::update()
{
    AP::hxky_uart().update();

    // 处理 0x31 航点指令
    if (AP::hxky_uart().has_new_0x31()) {
        const FD1_msg_0x31& msg = AP::hxky_uart().get_msg_0x31();
        AP::hxky_uart().send_reply(msg._msg_1.content.msg.cmd_type);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "-> New Target WP <-");

        int32_t lat_in = (double)msg._msg_1.content.msg.target_lat * msg.SF_LAT * 1e7;
        int32_t lng_in = (double)msg._msg_1.content.msg.target_lng * msg.SF_LNG * 1e7;
        int32_t alt_in = (msg._msg_1.content.msg.target_alt - 1000) * 100;
        Location temp_loc = Location(lat_in, lng_in, alt_in, Location::AltFrame::ABSOLUTE);

        if (msg._msg_1.content.msg.alt_type == 0) {
            temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABOVE_HOME);
        }
        if (msg._msg_1.content.msg.alt_type == 1) {
            temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABSOLUTE);
        }
        if (msg._msg_1.content.msg.alt_type == 2) {
            temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABOVE_TERRAIN);
        }
        if (msg._msg_1.content.msg.alt_type == 3) {
            temp_loc.set_alt_cm(alt_in, Location::AltFrame::ABSOLUTE);
        }

        float lat = lat_in;
        float lng = lng_in;
        float alt = alt_in * 0.01f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "-> [%f, %f, %f] <-", lng, lat, alt);

        if (plane.control_mode == &plane.mode_guided) {
            ;
        } else if (plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "-> In GUIDED <-");
            plane.control_mode->handle_guided_request(temp_loc);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Err, Can't to GUIDED");
        }
        AP::hxky_uart().clear_0x31();
    }

    // 处理 0x33 回收/自毁指令
    if (AP::hxky_uart().has_new_0x33()) {
        const FD1_msg_0x33& msg = AP::hxky_uart().get_msg_0x33();
        AP::hxky_uart().send_reply(msg._msg_1.content.msg.cmd_type);
        AP::hxky_uart().clear_0x33();
    }

    // 处理 0x36 进入攻击指令
    if (AP::hxky_uart().has_new_0x36()) {
        const FD1_msg_0x36& msg = AP::hxky_uart().get_msg_0x36();
        AP::hxky_uart().send_reply(msg._msg_1.content.msg.cmd_type);
        AP::hxky_uart().clear_0x36();
    }

    // 处理 0x37 退出攻击指令
    if (AP::hxky_uart().has_new_0x37()) {
        const FD1_msg_0x37& msg = AP::hxky_uart().get_msg_0x37();
        AP::hxky_uart().send_reply(msg._msg_1.content.msg.cmd_type);
        AP::hxky_uart().clear_0x37();
    }
}

void Uart::send_0x11()
{
    float temp_hagl = 0.0f;
    if (!plane.ahrs.get_hagl(temp_hagl)) {
        temp_hagl = 0.0f;
    }
    float temp_asp = plane.airspeed.get_airspeed();

    struct HXKY_Uart_0x11_data data;
    data.gps_lng = ((float)plane.gps.location().lng) / AP::hxky_uart().get_sf_lng();
    data.gps_lat = ((float)plane.gps.location().lat) / AP::hxky_uart().get_sf_lat();
    data.relative_alt = temp_hagl;
    data.absolute_alt = 0;
    data.baro_alt = 0;
    data.pitch_angle = wrap_180_cd(plane.ahrs.pitch_sensor);
    data.roll_angle = wrap_180_cd(plane.ahrs.roll_sensor);
    data.yaw_angle = wrap_360_cd(plane.ahrs.yaw_sensor);
    data.airspeed = temp_asp;
    data.vel_n = 0;
    data.vel_e = 0;
    data.vel_d = 0;
    data.rest_time = 0;
    data.status = 0;
    data.gps_count = 0;
    data.pos_source = 0;
    data.flight_mode = 0;
    data.time2000 = 0;
    data.vel_lat = 0;
    data.vel_lng = 0;
    data.vel_alt = 0;
    data.pitch_rate = 0;
    data.roll_rate = 0;
    data.yaw_rate = 0;

    AP::hxky_uart().send_0x11(data);
}

void Uart::send_0x22()
{
    bool takeoff = plane.is_flying();
    AP::hxky_uart().send_0x22(takeoff ? 0 : 1);
}

// 兼容旧接口：直接访问底层库的缩放因子
float Uart::get_sf_lng(void) const
{
    return AP::hxky_uart().get_sf_lng();
}

float Uart::get_sf_lat(void) const
{
    return AP::hxky_uart().get_sf_lat();
}
