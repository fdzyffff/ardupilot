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
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NET, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Uart NET init");
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
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        read_byte(temp);
    }
}

void Uart::read_byte(uint8_t temp)
{
    uart_msg_diaocang_cast.parse(temp);
    if (uart_msg_diaocang_cast._msg_1.updated) {
        handle_info_diaocang();
        uart_msg_diaocang_cast._msg_1.updated = false;
    }

    uart_msg_huokong_cast.parse(temp);
    if (uart_msg_huokong_cast._msg_1.updated) {
        handle_info_huokong();
        uart_msg_huokong_cast._msg_1.updated = false;
    }

    uart_msg_shuangmu_cast.parse(temp);
    if (uart_msg_shuangmu_cast._msg_1.updated) {
        handle_info_shuangmu();
        uart_msg_shuangmu_cast._msg_1.updated = false;
    }

    uart_msg_waibu_single.parse(temp);
    if (uart_msg_waibu_single._msg_1.updated) {
        handle_info_waibu();
        uart_msg_waibu_single._msg_1.updated = false;
    }
}

void Uart::write_uart()
{
    send_hangdong();
}

void Uart::handle_info_diaocang()
{
    for (uint8_t i_target = 1; i_target <=6; i_target++) {
        if (uart_msg_diaocang_cast._msg_1.content.msg.status_1 & (uint16_t)(1<<(6+i_target)))
        // 目标i跟踪好
        {
            float gimbal_yaw = uart_msg_diaocang_cast._msg_1.content.msg.gimbal_yaw;
            float gimbal_pitch = uart_msg_diaocang_cast._msg_1.content.msg.gimbal_pitch;
            float target_yaw = uart_msg_diaocang_cast._msg_1.content.msg.target[i_target-1].yaw;
            float target_pitch = uart_msg_diaocang_cast._msg_1.content.msg.target[i_target-1].pitch;

            Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
            Matrix3f tmp_target_cam_m;
            tmp_target_cam_m.from_euler(0.0f, radians(target_pitch), radians(target_yaw));
            Matrix3f tmp_cam_frame_m;
            tmp_cam_frame_m.from_euler(radians(0.0f), gimbal_pitch, gimbal_yaw);
            Matrix3f tmp_target_frame_m = tmp_cam_frame_m*tmp_target_cam_m;
            Vector3f ef_unit = tmp_target_frame_m*target_unit;

            // static uint32_t last_info_ms = millis();
            // if (millis() - last_info_ms > 1000) {
            //     last_info_ms = millis();
            //     gcs().send_text(MAV_SEVERITY_INFO, "KKKKKK (%f, %f, %f)", ef_unit.x, ef_unit.y, ef_unit.z);
            //     gcs().send_text(MAV_SEVERITY_INFO, "VVVVVV (%f, %f, %f)", degrees(_roll), degrees(_pitch), degrees(_yaw));
            // }

            float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
            float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));

            if (plane.uattack._Target_ptr_cam_FP847 != nullptr) {
                // plane.uattack._Target_ptr_cam_FP847->handle_info(wrap_360(gimbal_yaw + target_yaw), wrap_180(gimbal_pitch + target_pitch));
                plane.uattack._Target_ptr_cam_FP847->handle_info(angle_yaw, angle_pitch);
            }
            break;
        }
    }

    cali_last_ms = millis();
    cali_last_ms_global = get_time_ms(uart_msg_diaocang_cast._msg_1.content.msg.time);
}

void Uart::handle_info_huokong()
{

}

void Uart::handle_info_shuangmu()
{

}

void Uart::handle_info_waibu()
{
    Location base_loc;
    base_loc.lat = uart_msg_waibu_single._msg_1.content.msg.base_loc.lat;
    base_loc.lng = uart_msg_waibu_single._msg_1.content.msg.base_loc.lng;
    if ((uart_msg_waibu_single._msg_1.content.msg.base_loc.quadrant &0xF) == 0x02) {
        base_loc.lat = -base_loc.lat;
    }
    if ((uart_msg_waibu_single._msg_1.content.msg.base_loc.quadrant &0xF0) == 0x20) {
        base_loc.lng = -base_loc.lng;
    }
    int32_t tmp_alt_cm = uart_msg_waibu_single._msg_1.content.msg.base_loc.abs_alt;
    base_loc.set_alt_cm(tmp_alt_cm * 100, Location::AltFrame::ABSOLUTE);
    for (uint8_t i_target = 1; i_target <=6; i_target++) {
        float target_dist = ((float)uart_msg_waibu_single._msg_1.content.msg.target[i_target-1].R) * 0.1;
        float target_yaw = ((float)uart_msg_waibu_single._msg_1.content.msg.target[i_target-1].A) * 0.01f;
        float target_pitch = ((float)uart_msg_waibu_single._msg_1.content.msg.target[i_target-1].E) * 0.01f;;
        Vector3f base_vec = Vector3f(target_dist, 0.0f, 0.0f);
        Matrix3f tmp_target_base_m;
        tmp_target_base_m.from_euler(0.0f, radians(target_pitch), radians(target_yaw));
        Vector3f earch_vec = tmp_target_base_m * base_vec;
        Location target_loc = base_loc;
        target_loc.offset(Vector3p(earch_vec.x, earch_vec.y, earch_vec.z));
        if (plane.uattack._Target_ptr_loc != nullptr) {
            plane.uattack._Target_ptr_loc->set_target_location(target_loc);
        }
        break;
    }
}


void Uart::send_hangdong()
{
    // check send condition
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    if (now - last_ms < 50) {
        return;
    }
    last_ms = now;

    if (get_port() == nullptr) {return;}

    set_tims_ms(uart_msg_hangdong_cast._msg_1.content.msg.time);
    uart_msg_hangdong_cast._msg_1.content.msg.status_1 = 0b1111110;
    uart_msg_hangdong_cast._msg_1.content.msg.error_1 = 0;
    uart_msg_hangdong_cast._msg_1.content.msg.bearing = (uint32_t)(wrap_360(AP::gps().ground_course()) * 10000.f);
    uart_msg_hangdong_cast._msg_1.content.msg.yaw = (int32_t)(degrees(AP::ahrs().get_yaw()) * 10000.f);
    uart_msg_hangdong_cast._msg_1.content.msg.pitch = (int32_t)(degrees(AP::ahrs().get_pitch()) * 10000.f);
    uart_msg_hangdong_cast._msg_1.content.msg.roll = (int32_t)(degrees(AP::ahrs().get_roll()) * 10000.f);

    Matrix3f tmp_body_yaw_m;
    tmp_body_yaw_m.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), 0.0f);
    Vector3f earth_rate = tmp_body_yaw_m * AP::ahrs().get_gyro();
    uart_msg_hangdong_cast._msg_1.content.msg.yaw_rate = (int16_t)(degrees(earth_rate.z) * 100.f);
    uart_msg_hangdong_cast._msg_1.content.msg.pitch_rate = (int16_t)(degrees(earth_rate.y) * 100.f);
    uart_msg_hangdong_cast._msg_1.content.msg.roll_rate = (int16_t)(degrees(earth_rate.x) * 100.f);
    if (plane.airspeed.get_airspeed() > 0) {
        uart_msg_hangdong_cast._msg_1.content.msg.airspeed = (uint16_t)(plane.airspeed.get_airspeed());
    } else {
        uart_msg_hangdong_cast._msg_1.content.msg.airspeed = 0;
    }
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        uart_msg_hangdong_cast._msg_1.content.msg.ground_speed = (uint16_t)(AP::gps().ground_speed());
        uart_msg_hangdong_cast._msg_1.content.msg.ground_course = (uint32_t)(wrap_360(AP::gps().ground_course()) * 10000.f);
        if (plane.current_loc.lat > 0) {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lat = plane.current_loc.lat/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant &= 0xF0;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant += 0x01;
        } else {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lat = -plane.current_loc.lat/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant &= 0xF0;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant += 0x02;
        }
        if (plane.current_loc.lng > 0) {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lng = plane.current_loc.lng/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant &= 0x0F;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant += 0x10;
        } else {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lng = -plane.current_loc.lng/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant &= 0x0F;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant += 0x20;
        }
        int32_t tmp_alt_cm = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.current_loc.abs_alt = 0;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt_cm)) {
            if (tmp_alt_cm > 0) {
                uart_msg_hangdong_cast._msg_1.content.msg.current_loc.abs_alt = tmp_alt_cm/100;
            }
        }
        uart_msg_hangdong_cast._msg_1.content.msg.year = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.month = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.day = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.hour = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.minute = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.second = 0;
    }
    uart_msg_hangdong_cast._msg_1.content.msg.yaw_corr = 0;
    uart_msg_hangdong_cast._msg_1.content.msg.pitch_corr = 0;

    uart_msg_hangdong_cast.make_sum();
    get_port()->write(uart_msg_hangdong_cast._msg_1.content.data, uart_msg_hangdong_cast._msg_1.content.msg.length);
    // gcs().send_text(MAV_SEVERITY_INFO, "r: %0.1f, p: %0.1f, y: %0.1f", degrees(earth_rate.x), degrees(earth_rate.y), degrees(earth_rate.z));
}

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
    time.bits.ms = ms_global%(24 * 60 * 60 * 1000);
    time.bits.sec = (ms_global/1000)%(24 * 60 * 60);
    time.bits.min = (ms_global/(60 * 1000))%(24 * 60);
    time.bits.h = (ms_global/(60 * 60 * 1000))%(24);
    time.bits.day = (ms_global/(24 * 60 * 60 * 1000));
}
