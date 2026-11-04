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
    _port_rtkrid = nullptr;
    _last_rid_ms = 0;
    _rid_text_start_ms = 0;
    _last_rtkrid_identity_ms = 0;
    _rtkrid_cfg_reported = false;

    // 协议号 50：老 RID 模块（203 字节二进制国标帧）
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_RID, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RID init");
    }

    // 协议号 51：RID + RTK 一体化板卡（ASCII 命令），独立 TTL 串口，57600
    _port_rtkrid = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_RTKRID, 0);
    if (_port_rtkrid != nullptr) {
        // 文档 3.2 要求 57600（SERIALn_BAUD=57 → baudrate() 返回 57000）
        const AP_SerialManager::UARTState *state = AP::serialmanager().find_protocol_instance(AP_SerialManager::SerialProtocol_RTKRID, 0);
        if ((state != nullptr) && (state->baudrate() != 57000) && (state->baudrate() != 57600)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RTKRID baud must be 57600");
        }
        _msg_RTKRID.init(millis());
        gcs().send_text(MAV_SEVERITY_WARNING, "RTKRID init");
    }
}

void Uart::update()
{
    read_uart();
    write_uart();
}

void Uart::read_uart()
{
    if (_port_rtkrid == nullptr) {
        return;
    }

    // 逐字节喂行缓冲：模块上电日志（+EIND:128/$STATUS,5001）与 GGA 由 handle_line 丢弃，
    // 只认 '*' 开头的命令应答
    while (_port_rtkrid->available() > 0) {
        _msg_RTKRID.parse((uint8_t)_port_rtkrid->read());
    }
}

void Uart::pack_rid_msg()
{
    uint8_t cmd_mask[4] = {0};
    cmd_mask[0] = 0b11111111;
    cmd_mask[1] = 0b11111111;
    cmd_mask[2] = 0b11111111;
    cmd_mask[3] = 0b00000000;
    _msg_RID.make_init();
    uint8_t insert_data[20];

    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    if (_msg_RID.have_msg_id(1, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        char char_data[20] = {'0'};
        if (AP::fd_data().get_serial_number(char_data)) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[0], 20);
            _msg_RID.insert_msg(1, insert_data);
        }
    }

    //0x40 002 M 实名登记标志
    if (_msg_RID.have_msg_id(2, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        char char_data[20] = {'0'};
        if (AP::fd_data().get_uas_number(char_data)) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[12], 8);
            _msg_RID.insert_msg(2, insert_data);
        }
    }

    //0x20 003 O 民用无人驾驶航空器系统运行类别
    if (_msg_RID.have_msg_id(3, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(3, insert_data);
    }

    //0x10 004 M 民用无人驾驶航空器分类
    if (_msg_RID.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(4, insert_data);
    }

    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    if (_msg_RID.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(5, insert_data);
    }

    //0x04 006 M 民用无人驾驶航空器遥控站位置
    if (_msg_RID.have_msg_id(6, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
        if (gcs_msg.longitude != 0 && gcs_msg.latitude != 0) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&gcs_msg.longitude, 4);
            memcpy((uint8_t *)&insert_data[4], (uint8_t *)&gcs_msg.latitude, 4);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 4);
            memset((uint8_t *)&insert_data[4], 0xFF, 4);
        }
        _msg_RID.insert_msg(6, insert_data);
    }


    //0x02 007 M 民用无人驾驶航空器遥控站高度
    if (_msg_RID.have_msg_id(7, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
        uint16_t out_alt = ((int16_t)gcs_msg.altitude + 1000)*2;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_alt, 2);
        _msg_RID.insert_msg(7, insert_data);
    }


    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    if (_msg_RID.have_msg_id(8, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&copter.current_loc.lng, 4);
            memcpy((uint8_t *)&insert_data[4], (uint8_t *)&copter.current_loc.lat, 4);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 4);
            memset((uint8_t *)&insert_data[4], 0xFF, 4);
        }
        _msg_RID.insert_msg(8, insert_data);
    }

    //0x40 009 M 航迹角
    if (_msg_RID.have_msg_id(9, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            uint16_t out = (uint16_t)(wrap_360(AP::gps().ground_course()) * 10.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 2);
        }
        _msg_RID.insert_msg(9, insert_data);
    }

    //0x20 010 M 地速
    if (_msg_RID.have_msg_id(10, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            uint16_t out = (uint16_t)(AP::ahrs().groundspeed() * 10.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 2);
        }
        _msg_RID.insert_msg(10, insert_data);
    }

    //0x10 011 O 相对高度
    if (_msg_RID.have_msg_id(11, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            float home_d = 0.0f;
            AP::ahrs().get_relative_position_D_home(home_d);
            if (!is_zero(home_d)) {
                uint16_t out = (uint16_t)((-home_d + 9000.f) * 2.0f);
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
            } else {
                memset((uint8_t *)&insert_data[0], 0x00, 2);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_RID.insert_msg(11, insert_data);
    }

    //0x08 012 O 垂直速度
    if (_msg_RID.have_msg_id(12, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            float vert_d = 0.0f;
            if (AP::ahrs().get_vert_pos_rate_D(vert_d)) {
                uint8_t out = (uint16_t)(constrain_float(fabsf(vert_d) * 2.0f, 0.0f, 127.f));
                if (vert_d > 0.0f) {
                    out += 128;
                }
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
            } else {
                memset((uint8_t *)&insert_data[0], 0xFF, 1);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 1);
        }
        _msg_RID.insert_msg(12, insert_data);
    }

    //0x04 013 M 大地高度
    if (_msg_RID.have_msg_id(13, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            int32_t abs_alt_cm = 0.0f;
            if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
                uint16_t out = (uint16_t)((((float)abs_alt_cm)*0.01f + 1000.f) * 2.0f);
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
            } else {
                memset((uint8_t *)&insert_data[0], 0x00, 2);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_RID.insert_msg(13, insert_data);
    }

    //0x02 014 O 气压高度
    if (_msg_RID.have_msg_id(14, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            int32_t abs_alt_cm = 0.0f;
            if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
                uint16_t out = (uint16_t)((((float)abs_alt_cm)*0.01f + 1000.f) * 2.0f);
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
            } else {
                memset((uint8_t *)&insert_data[0], 0x00, 2);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_RID.insert_msg(14, insert_data);
    }

    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    if (_msg_RID.have_msg_id(15, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.ap.land_complete) {
            insert_data[0] = 1;
        }
        if (!copter.ap.land_complete) {
            insert_data[0] = 2;
        }
        if (copter.failsafe.radio || copter.failsafe.gcs || copter.failsafe.ekf) {
            insert_data[0] = 3;
        }
        _msg_RID.insert_msg(15, insert_data);
    }

    //0x40 016 M 坐标系类型
    if (_msg_RID.have_msg_id(16, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        _msg_RID.insert_msg(16, insert_data);
    }

    //0x20 017 M 水平精度
    if (_msg_RID.have_msg_id(17, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 11;
            }
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 12;
            }
        }
        _msg_RID.insert_msg(17, insert_data);
    }

    //0x10 018 M 垂直精度
    if (_msg_RID.have_msg_id(18, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 4;
            }
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 1;
            }
        }
        _msg_RID.insert_msg(18, insert_data);
    }

    //0x08 019 M 速度精度
    if (_msg_RID.have_msg_id(19, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 3;
            }
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 4;
            }
        }
        _msg_RID.insert_msg(19, insert_data);
    }

    //0x04 020 M 时间戳
    if (_msg_RID.have_msg_id(20, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        uint64_t out_ms = 0;
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            out_ms = (uint64_t)AP::gps().time_week() * 86400LLU * 7000LLU + (uint64_t)AP::gps().time_week_ms();
        }
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_ms, 6);
        _msg_RID.insert_msg(20, insert_data);
    }

    //0x02 021 M 时间戳精度
    if (_msg_RID.have_msg_id(21, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            insert_data[0] = 8;
        }
        _msg_RID.insert_msg(21, insert_data);
    }
    //0x01       扩展标志位
}


void Uart::pack_rid_msg_test()
{
    uint8_t cmd_mask[4] = {0};
    cmd_mask[0] = 0b11111111;
    cmd_mask[1] = 0b11111111;
    cmd_mask[2] = 0b11111111;
    cmd_mask[3] = 0b00000000;
    _msg_RID.make_init();
    uint8_t insert_data[20];

    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    if (_msg_RID.have_msg_id(1, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        char char_data[21] = "ABCDEFGHIJKLMNOPQRST";
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[0], 20);
        _msg_RID.insert_msg(1, insert_data);
    }

    //0x40 002 M 实名登记标志
    if (_msg_RID.have_msg_id(2, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        char char_data[21] = "tsrqponmlkjihgfedcba";
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[12], 8);
        _msg_RID.insert_msg(2, insert_data);
    }

    //0x20 003 O 民用无人驾驶航空器系统运行类别
    if (_msg_RID.have_msg_id(3, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(3, insert_data);
    }

    //0x10 004 M 民用无人驾驶航空器分类
    if (_msg_RID.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(4, insert_data);
    }

    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    if (_msg_RID.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(5, insert_data);
    }

    //0x04 006 M 民用无人驾驶航空器遥控站位置
    if (_msg_RID.have_msg_id(6, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t test_lon = 1234567890;
        int32_t test_lat = -1234567890;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_lon, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&test_lat, 4);
        _msg_RID.insert_msg(6, insert_data);
    }


    //0x02 007 M 民用无人驾驶航空器遥控站高度
    if (_msg_RID.have_msg_id(7, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        uint16_t test_altitude = 2500;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_altitude, 2);
        _msg_RID.insert_msg(7, insert_data);
    }


    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    if (_msg_RID.have_msg_id(8, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t test_lon = 1234567890;
        int32_t test_lat = -1234567890;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_lon, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&test_lat, 4);
        _msg_RID.insert_msg(8, insert_data);
    }

    //0x40 009 M 航迹角
    if (_msg_RID.have_msg_id(9, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float test_yaw = 250.f;
        uint16_t out = (uint16_t)(wrap_360(test_yaw) * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_RID.insert_msg(9, insert_data);
    }

    //0x20 010 M 地速
    if (_msg_RID.have_msg_id(10, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float test_gspd = 250.f;
        uint16_t out = (uint16_t)(test_gspd * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_RID.insert_msg(10, insert_data);
    }

    //0x10 011 O 相对高度
    if (_msg_RID.have_msg_id(11, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float home_d = 250.0f;
        AP::ahrs().get_relative_position_D_home(home_d);
        if (!is_zero(home_d)) {
            uint16_t out = (uint16_t)((-home_d + 9000.f) * 2.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_RID.insert_msg(11, insert_data);
    }

    //0x08 012 O 垂直速度
    if (_msg_RID.have_msg_id(12, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float vert_d = 25.0f;
        if (AP::ahrs().get_vert_pos_rate_D(vert_d)) {
            uint8_t out = (uint16_t)(constrain_float(fabsf(vert_d) * 2.0f, 0.0f, 127.f));
            if (vert_d > 0.0f) {
                out += 127;
            }
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 1);
        }
        _msg_RID.insert_msg(12, insert_data);
    }

    //0x04 013 M 大地高度
    if (_msg_RID.have_msg_id(13, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t abs_alt_cm = 250.0f;
        uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_RID.insert_msg(13, insert_data);
    }

    //0x02 014 O 气压高度
    if (_msg_RID.have_msg_id(14, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t abs_alt_cm = 250.0f;
        uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_RID.insert_msg(14, insert_data);
    }

    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    if (_msg_RID.have_msg_id(15, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(15, insert_data);
    }

    //0x40 016 M 坐标系类型
    if (_msg_RID.have_msg_id(16, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_RID.insert_msg(16, insert_data);
    }

    //0x20 017 M 水平精度
    if (_msg_RID.have_msg_id(17, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 12;
        _msg_RID.insert_msg(17, insert_data);
    }

    //0x10 018 M 垂直精度
    if (_msg_RID.have_msg_id(18, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 6;
        _msg_RID.insert_msg(18, insert_data);
    }

    //0x08 019 M 速度精度
    if (_msg_RID.have_msg_id(19, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 4;
        _msg_RID.insert_msg(19, insert_data);
    }

    //0x04 020 M 时间戳
    if (_msg_RID.have_msg_id(20, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        uint64_t out_ms = 25000000000000LLU;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_ms, 6);
        _msg_RID.insert_msg(20, insert_data);
    }

    //0x02 021 M 时间戳精度
    if (_msg_RID.have_msg_id(21, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 8;
        _msg_RID.insert_msg(21, insert_data);
    }
    //0x01       扩展标志位
}

void Uart::write_uart()
{
    write_uart_rid();
    write_uart_rtkrid();
}

void Uart::write_uart_rid()
{
    if (get_port() == nullptr) {return;}
    if (millis() - _last_rid_ms > 1000) {
        _last_rid_ms = millis();
        pack_rid_msg();
        // pack_rid_msg_test();
        get_port()->write(_msg_RID._msg_1.content.data, _msg_RID._msg_1.length);
        pack_rid_text();
    }
}

// 每次调用最多发一条命令：uart.update() 挂在 100 Hz 的 userhook_FastLoop（Copter.cpp:249）上，
// 单条命令 ≤30 字节，一轮探测 burst（2 读 + 6 写）在 80 ms 内发完，不会撑爆 256 字节 TX 缓冲
void Uart::write_uart_rtkrid()
{
    if (_port_rtkrid == nullptr) {
        return;
    }

    const uint32_t now_ms = millis();

    // 身份来源可由地面站运行时改写（FD_DATA::set_serial_number），1 Hz 刷新一次即可
    if ((now_ms - _last_rtkrid_identity_ms) >= 1000) {
        _last_rtkrid_identity_ms = now_ms;
        char serial_number[21] = {0};
        char uas_number[21] = {0};
        AP::fd_data().get_serial_number(serial_number);
        AP::fd_data().get_uas_number(uas_number);
        _msg_RTKRID.set_identity(serial_number, uas_number);
    }

    char buf[FD1_MSG_RTKRID_CMD_MAX];
    const uint16_t len = _msg_RTKRID.get_pending(buf, sizeof(buf), get_ua_status());
    if (len > 0) {
        _port_rtkrid->write((uint8_t *)buf, len);
    }

    if (_msg_RTKRID.configured() && !_rtkrid_cfg_reported) {
        _rtkrid_cfg_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO, "RTKRID cfg done");
    }
}

// 运行状态：1 = 地面，2 = 空中，3 = 故障（口径与 pack_rid_text() 一致）
uint8_t Uart::get_ua_status() const
{
    if (!copter.motors->armed()) {
        return 1;
    }
    if (!copter.position_ok()) {
        return 3;
    }
    if (copter.ap.land_complete) {
        return 1;
    }
    return 2;
}

// ASCII plaintext RID messages
void Uart::pack_rid_text()
{
    if (_rid_text_start_ms == 0) {
        _rid_text_start_ms = millis();
    }

    const bool static_active = (millis() - _rid_text_start_ms < 15000);

    char buf[128];

    // UA_STATUS: 1Hz always  1=ground 2=air 3=fault
    //   not armed -> ground unconditionally
    uint8_t ua_status = 2;
    if (!copter.motors->armed()) {
        ua_status = 1;
    } else if (!copter.position_ok()) {
        ua_status = 3;
    } else if (copter.ap.land_complete) {
        ua_status = 1;
    }
    snprintf(buf, sizeof(buf), "*UA_STATUS_W:%d#\r\n", ua_status);
    get_port()->write((uint8_t *)buf, strlen(buf));

    if (!static_active) {
        return;
    }

    // UAS_ID: serial number
    char serial[20] = {0};
    if (AP::fd_data().get_serial_number(serial)) {
        snprintf(buf, sizeof(buf), "*UAS_ID_W:%s#\r\n", serial);
        get_port()->write((uint8_t *)buf, strlen(buf));
    }

    // UASID_RNM: real-name registration flag (8 bytes from uas_number[12..19])
    char uas_num[20] = {0};
    if (AP::fd_data().get_uas_number(uas_num)) {
        snprintf(buf, sizeof(buf), "*UASID_RNM_W:%.8s#\r\n", &uas_num[12]);
        get_port()->write((uint8_t *)buf, strlen(buf));
    }

    // UA_TYPE: fixed 2
    snprintf(buf, sizeof(buf), "*UA_TYPE_W:2#\r\n");
    get_port()->write((uint8_t *)buf, strlen(buf));

    // UA_CLASS: fixed 3
    snprintf(buf, sizeof(buf), "*UA_CLASS_W:3#\r\n");
    get_port()->write((uint8_t *)buf, strlen(buf));

    // UA_CATEGORY: fixed 3
    snprintf(buf, sizeof(buf), "*UA_CATEGORY_W:3#\r\n");
    get_port()->write((uint8_t *)buf, strlen(buf));

    // ATTRIBUTION: fixed 2
    snprintf(buf, sizeof(buf), "*ATTRIBUTION_R:2#\r\n");
    get_port()->write((uint8_t *)buf, strlen(buf));
}
