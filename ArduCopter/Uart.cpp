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
    _port = plane.serial_manager.find_serial(AP_SerialManager::SerialProtocol_MISSION, 0);
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
    // if (get_port() == nullptr) {return;}
    // while (get_port()->available()>0) {
    //     uint8_t temp = get_port()->read();
    // }
}

void Uart::pack_uom_msg()
{
    make_init();
    char insert_data[20];
    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    memset(insert_data, 0, sizeof(insert_data));
    if (AP::fd_data().get_serial_number(insert_data)) {
        _msg_UOM.insert_msg(1, insert_data);
    }

    //0x40 002 M 实名登记标志
    memset(insert_data, 0, sizeof(insert_data));
    if (AP::fd_data().get_uas_number(insert_data)) {
        _msg_UOM.insert_msg(2, insert_data);
    }

    //0x20 003 O 民用无人驾驶航空器系统运行类别
    memset(insert_data, 0, sizeof(insert_data));
    insert_data[0] = 1;
    _msg_UOM.insert_msg(3, insert_data);

    //0x10 004 M 民用无人驾驶航空器分类
    memset(insert_data, 0, sizeof(insert_data));
    insert_data[0] = 1;
    _msg_UOM.insert_msg(4, insert_data);

    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    memset(insert_data, 0, sizeof(insert_data));
    insert_data[0] = 1;
    _msg_UOM.insert_msg(5, insert_data);

    //0x04 006 M 民用无人驾驶航空器遥控站位置
    memset(insert_data, 0, sizeof(insert_data));
    mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
    if (gcs_msg.longitude != 0 && gcs_msg.latitude != 0) {
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&gcs_msg.longitude, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&gcs_msg.latitude, 4);
    } else {
        memcpy((uint8_t *)&insert_data[0], 0xFF, 4);
        memcpy((uint8_t *)&insert_data[4], 0xFF, 4);
    }
    _msg_UOM.insert_msg(6, insert_data);

    //0x02 007 M 民用无人驾驶航空器遥控站高度
    memset(insert_data, 0, sizeof(insert_data));
    mavlink_zfjl_gcs_heartbeat_t &gcs_msg = AP::fd_data().get_gcs_heartbeat_msg();
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&gcs_msg.altitude, 2);
    _msg_UOM.insert_msg(7, insert_data);

    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    memset(insert_data, 0, sizeof(insert_data));
    if (copter.position_ok()) {
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&copter.current_loc.lng, 4);
        memcpy((uint8_t *)&insert_data[4], (uint8_t *)&copter.current_loc.lat, 4);
    } else {
        memcpy((uint8_t *)&insert_data[0], 0xFF, 4);
        memcpy((uint8_t *)&insert_data[4], 0xFF, 4);
    }
    _msg_UOM.insert_msg(8, insert_data);

    //0x40 009 M 航迹角
    memset(insert_data, 0, sizeof(insert_data));
    if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
        uint16_t out = (uint16_t)(wrap_360(AP::gps().ground_course()) * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
    } else {
        memcpy((uint8_t *)&insert_data[0], 0xFF, 2);
    }
    _msg_UOM.insert_msg(9, insert_data);

    //0x20 010 M 地速
    memset(insert_data, 0, sizeof(insert_data));
    if (copter.position_ok()) {
        uint16_t out = (uint16_t)(AP::ahrs().groundspeed() * 10.0f);
        memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
    } else {
        memcpy((uint8_t *)&insert_data[0], 0xFF, 2);
    }
    _msg_UOM.insert_msg(10, insert_data);

    //0x10 011 O 相对高度
    memset(insert_data, 0, sizeof(insert_data));
    if (copter.position_ok()) {
        float home_d = 0.0f;
        if (AP::ahrs().get_relative_position_D_home(home_d)) {
            uint16_t out = (uint16_t)((-home_d + 9000.f) * 2.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memcpy((uint8_t *)&insert_data[0], 0x00, 2);
        }
    } else {
        memcpy((uint8_t *)&insert_data[0], 0x00, 2);
    }
    _msg_UOM.insert_msg(11, insert_data);

    //0x08 012 O 垂直速度
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
            memcpy((uint8_t *)&insert_data[0], 0xFF, 1);
        }
    } else {
        memcpy((uint8_t *)&insert_data[0], 0xFF, 1);
    }
    _msg_UOM.insert_msg(12, insert_data);

    //0x04 013 M 大地高度
    memset(insert_data, 0, sizeof(insert_data));
    if (copter.position_ok()) {
        float abs_alt_cm = 0.0f;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
            uint16_t out = (uint16_t)((abs_alt_cm + 1000.f) * 2.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memcpy((uint8_t *)&insert_data[0], 0x00, 2);
        }
    } else {
        memcpy((uint8_t *)&insert_data[0], 0x00, 2);
    }
    _msg_UOM.insert_msg(13, insert_data);

    //0x02 014 O 气压高度
    memset(insert_data, 0, sizeof(insert_data));
    if (copter.position_ok()) {
        float abs_alt_cm = 0.0f;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, abs_alt_cm)) {
            uint16_t out = (uint16_t)((abs_alt_cm + 1000.f) * 2.0f);
            memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 2);
        } else {
            memcpy((uint8_t *)&insert_data[0], 0x00, 2);
        }
    } else {
        memcpy((uint8_t *)&insert_data[0], 0x00, 2);
    }
    _msg_UOM.insert_msg(14, insert_data);

    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    if (copter.ap.land_complete) {
        out = 1;
    }
    if (!copter.ap.land_complete) {
        out = 2;
    }
    if (copter.failsafe.radio || copter.failsafe.gcs || copter.failsafe.ekf) {
        out = 3;
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(15, insert_data);

    //0x40 016 M 坐标系类型
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(16, insert_data);

    //0x20 017 M 水平精度
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    if (copter.position_ok()) {
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            out = 11;
        }
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
            out = 12;
        }
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(17, insert_data);

    //0x10 018 M 垂直精度
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    if (copter.position_ok()) {
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            out = 4;
        }
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
            out = 1;
        }
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(18, insert_data);

    //0x08 019 M 速度精度
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    if (copter.position_ok()) {
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
            out = 3;
        }
        if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
            out = 4;
        }
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(19, insert_data);

    //0x04 020 M 时间戳
    memset(insert_data, 0, sizeof(insert_data));
    uint64_t out = 0;
    if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
        out = (uint64_t)AP::gps().status().time_week * 86400LLU * 7000LLU + (uint64_t)AP::gps().status().time_week_ms;
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out + 2, 6);
    _msg_UOM.insert_msg(20, insert_data);

    //0x02 021 M 时间戳精度
    memset(insert_data, 0, sizeof(insert_data));
    uint8_t out = 0;
    if (AP::gps().status() > AP_GPS::GPS_OK_FIX_3D) {
        out = 8;
    }
    memcpy((uint8_t *)&insert_data[0], (uint8_t *)&out, 1);
    _msg_UOM.insert_msg(21, insert_data);
    //0x01       扩展标志位
}

void Uart::write_uart()
{
    ;
}
