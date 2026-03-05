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
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_UOM, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UOM init");
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
    // if (get_port() == nullptr) {return;}
    // while (get_port()->available()>0) {
    //     uint8_t temp = get_port()->read();
    // }
}

void Uart::pack_uom_msg()
{
    uint8_t cmd_mask[4] = {0};
    cmd_mask[0] = 0b10000000;
    // cmd_mask[1] = 0b11111111;
    // cmd_mask[2] = 0b11111111;
    cmd_mask[3] = 0b00000000;
    _msg_UOM.make_init();
    uint8_t insert_data[20];

    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    if (_msg_UOM.have_msg_id(1, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        char char_data[20] = {'0'};
        if (AP::fd_data().get_serial_number(char_data)) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[0], 20);
            _msg_UOM.insert_msg(1, insert_data);
        }
    }

    //0x40 002 M 实名登记标志
    if (_msg_UOM.have_msg_id(2, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        char char_data[20] = {'0'};
        if (AP::fd_data().get_uas_number(char_data)) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[12], 8);
            _msg_UOM.insert_msg(2, insert_data);
        }
    }

    //0x20 003 O 民用无人驾驶航空器系统运行类别
    if (_msg_UOM.have_msg_id(3, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(3, insert_data);
    }

    //0x10 004 M 民用无人驾驶航空器分类
    if (_msg_UOM.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(4, insert_data);
    }

    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    if (_msg_UOM.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(5, insert_data);
    }

    //0x04 006 M 民用无人驾驶航空器遥控站位置
    if (_msg_UOM.have_msg_id(6, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
        if (gcs_msg.longitude != 0 && gcs_msg.latitude != 0) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&gcs_msg.longitude, 4);
            memcpy((uint8_t *)&insert_data[4], (uint8_t *)&gcs_msg.latitude, 4);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 4);
            memset((uint8_t *)&insert_data[4], 0xFF, 4);
        }
        _msg_UOM.insert_msg(6, insert_data);
    }


    //0x02 007 M 民用无人驾驶航空器遥控站高度
    if (_msg_UOM.have_msg_id(7, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
        uint16_t out_alt = ((int16_t)gcs_msg.altitude + 1000)*2;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_alt, 2);
        _msg_UOM.insert_msg(7, insert_data);
    }


    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    if (_msg_UOM.have_msg_id(8, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&copter.current_loc.lng, 4);
            memcpy((uint8_t *)&insert_data[4], (uint8_t *)&copter.current_loc.lat, 4);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 4);
            memset((uint8_t *)&insert_data[4], 0xFF, 4);
        }
        _msg_UOM.insert_msg(8, insert_data);
    }

    //0x40 009 M 航迹角
    if (_msg_UOM.have_msg_id(9, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            uint16_t out = (uint16_t)(wrap_360(AP::gps().ground_course()) * 10.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 2);
        }
        _msg_UOM.insert_msg(9, insert_data);
    }

    //0x20 010 M 地速
    if (_msg_UOM.have_msg_id(10, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            uint16_t out = (uint16_t)(AP::ahrs().groundspeed() * 10.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 2);
        }
        _msg_UOM.insert_msg(10, insert_data);
    }

    //0x10 011 O 相对高度
    if (_msg_UOM.have_msg_id(11, cmd_mask)) {
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
        _msg_UOM.insert_msg(11, insert_data);
    }

    //0x08 012 O 垂直速度
    if (_msg_UOM.have_msg_id(12, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            float vert_d = 0.0f;
            if (AP::ahrs().get_vert_pos_rate_D(vert_d)) {
                uint8_t out = (uint16_t)(constrain_float(fabsf(vert_d) * 2.0f, 0.0f, 127.f));
                if (vert_d > 0.0f) {
                    out += 127;
                }
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
            } else {
                memset((uint8_t *)&insert_data[0], 0xFF, 1);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0xFF, 1);
        }
        _msg_UOM.insert_msg(12, insert_data);
    }

    //0x04 013 M 大地高度
    if (_msg_UOM.have_msg_id(13, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            int32_t abs_alt_cm = 0.0f;
            if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
                uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
            } else {
                memset((uint8_t *)&insert_data[0], 0x00, 2);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_UOM.insert_msg(13, insert_data);
    }

    //0x02 014 O 气压高度
    if (_msg_UOM.have_msg_id(14, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        if (copter.position_ok()) {
            int32_t abs_alt_cm = 0.0f;
            if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
                uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
                memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
            } else {
                memset((uint8_t *)&insert_data[0], 0x00, 2);
            }
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_UOM.insert_msg(14, insert_data);
    }

    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    if (_msg_UOM.have_msg_id(15, cmd_mask)) {
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
        _msg_UOM.insert_msg(15, insert_data);
    }

    //0x40 016 M 坐标系类型
    if (_msg_UOM.have_msg_id(16, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        _msg_UOM.insert_msg(16, insert_data);
    }

    //0x20 017 M 水平精度
    if (_msg_UOM.have_msg_id(17, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 11;
            }
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 12;
            }
        }
        _msg_UOM.insert_msg(17, insert_data);
    }

    //0x10 018 M 垂直精度
    if (_msg_UOM.have_msg_id(18, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 4;
            }
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 1;
            }
        }
        _msg_UOM.insert_msg(18, insert_data);
    }

    //0x08 019 M 速度精度
    if (_msg_UOM.have_msg_id(19, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (copter.position_ok()) {
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
                insert_data[0] = 3;
            }
            if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
                insert_data[0] = 4;
            }
        }
        _msg_UOM.insert_msg(19, insert_data);
    }

    //0x04 020 M 时间戳
    if (_msg_UOM.have_msg_id(20, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        uint64_t out_ms = 0;
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            out_ms = (uint64_t)AP::gps().time_week() * 86400LLU * 7000LLU + (uint64_t)AP::gps().time_week_ms();
        }
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_ms, 6);
        _msg_UOM.insert_msg(20, insert_data);
    }

    //0x02 021 M 时间戳精度
    if (_msg_UOM.have_msg_id(21, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 0;
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            insert_data[0] = 8;
        }
        _msg_UOM.insert_msg(21, insert_data);
    }
    //0x01       扩展标志位
}


void Uart::pack_uom_msg_test()
{
    uint8_t cmd_mask[4] = {0};
    cmd_mask[0] = 0b11111111;
    cmd_mask[1] = 0b11111111;
    cmd_mask[2] = 0b11111111;
    cmd_mask[3] = 0b00000000;
    _msg_UOM.make_init();
    uint8_t insert_data[20];

    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    if (_msg_UOM.have_msg_id(1, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        char char_data[21] = "ABCDEFGHIJKLMNOPQRST";
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[0], 20);
        _msg_UOM.insert_msg(1, insert_data);
    }

    //0x40 002 M 实名登记标志
    if (_msg_UOM.have_msg_id(2, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        char char_data[21] = "tsrqponmlkjihgfedcba";
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&char_data[12], 8);
        _msg_UOM.insert_msg(2, insert_data);
    }

    //0x20 003 O 民用无人驾驶航空器系统运行类别
    if (_msg_UOM.have_msg_id(3, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(3, insert_data);
    }

    //0x10 004 M 民用无人驾驶航空器分类
    if (_msg_UOM.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(4, insert_data);
    }

    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    if (_msg_UOM.have_msg_id(4, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(5, insert_data);
    }

    //0x04 006 M 民用无人驾驶航空器遥控站位置
    if (_msg_UOM.have_msg_id(6, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t test_lon = 1234567890;
        int32_t test_lat = -1234567890;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_lon, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&test_lat, 4);
        _msg_UOM.insert_msg(6, insert_data);
    }


    //0x02 007 M 民用无人驾驶航空器遥控站高度
    if (_msg_UOM.have_msg_id(7, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));   
        uint16_t test_altitude = 2500;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_altitude, 2);
        _msg_UOM.insert_msg(7, insert_data);
    }


    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    if (_msg_UOM.have_msg_id(8, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t test_lon = 1234567890;
        int32_t test_lat = -1234567890;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&test_lon, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&test_lat, 4);
        _msg_UOM.insert_msg(8, insert_data);
    }

    //0x40 009 M 航迹角
    if (_msg_UOM.have_msg_id(9, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float test_yaw = 250.f;
        uint16_t out = (uint16_t)(wrap_360(test_yaw) * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_UOM.insert_msg(9, insert_data);
    }

    //0x20 010 M 地速
    if (_msg_UOM.have_msg_id(10, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float test_gspd = 250.f;
        uint16_t out = (uint16_t)(test_gspd * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_UOM.insert_msg(10, insert_data);
    }

    //0x10 011 O 相对高度
    if (_msg_UOM.have_msg_id(11, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        float home_d = 250.0f;
        AP::ahrs().get_relative_position_D_home(home_d);
        if (!is_zero(home_d)) {
            uint16_t out = (uint16_t)((-home_d + 9000.f) * 2.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memset((uint8_t *)&insert_data[0], 0x00, 2);
        }
        _msg_UOM.insert_msg(11, insert_data);
    }

    //0x08 012 O 垂直速度
    if (_msg_UOM.have_msg_id(12, cmd_mask)) {
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
        _msg_UOM.insert_msg(12, insert_data);
    }

    //0x04 013 M 大地高度
    if (_msg_UOM.have_msg_id(13, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t abs_alt_cm = 250.0f;
        uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_UOM.insert_msg(13, insert_data);
    }

    //0x02 014 O 气压高度
    if (_msg_UOM.have_msg_id(14, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        int32_t abs_alt_cm = 250.0f;
        uint16_t out = (uint16_t)(((float)abs_alt_cm + 1000.f) * 2.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        _msg_UOM.insert_msg(14, insert_data);
    }

    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    if (_msg_UOM.have_msg_id(15, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(15, insert_data);
    }

    //0x40 016 M 坐标系类型
    if (_msg_UOM.have_msg_id(16, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 1;
        _msg_UOM.insert_msg(16, insert_data);
    }

    //0x20 017 M 水平精度
    if (_msg_UOM.have_msg_id(17, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 12;
        _msg_UOM.insert_msg(17, insert_data);
    }

    //0x10 018 M 垂直精度
    if (_msg_UOM.have_msg_id(18, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 6;
        _msg_UOM.insert_msg(18, insert_data);
    }

    //0x08 019 M 速度精度
    if (_msg_UOM.have_msg_id(19, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 4;
        _msg_UOM.insert_msg(19, insert_data);
    }

    //0x04 020 M 时间戳
    if (_msg_UOM.have_msg_id(20, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        uint64_t out_ms = 25000000000000LLU;
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out_ms, 6);
        _msg_UOM.insert_msg(20, insert_data);
    }

    //0x02 021 M 时间戳精度
    if (_msg_UOM.have_msg_id(21, cmd_mask)) {
        memset(insert_data, 0, sizeof(insert_data));
        insert_data[0] = 8;
        _msg_UOM.insert_msg(21, insert_data);
    }
    //0x01       扩展标志位
}

void Uart::write_uart()
{
    if (get_port() == nullptr) {return;}
    if (millis() - _last_uom_ms > 1000) {
        _last_uom_ms = millis();
        // pack_uom_msg();
        pack_uom_msg_test();
        get_port()->write(_msg_UOM._msg_1.content.data, _msg_UOM._msg_1.length);
    }
}
