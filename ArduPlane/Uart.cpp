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
    time_check_ms = 0;
    time_gps_sec_last = 0;
    time_edge_local = 0;
    time_edge_count = 0;
    time_edge_err_sum = 0;
    pps_count = 0;
    pps_gpio = -1;
    sync_gpio = -1;
    timesync_printed = false;
    sync_pending = false;
    sync_last_us = 0;
    sync_count = 0;
}

// initialise
void Uart::init()
{
    _port = nullptr;

    // 时统 GPIO 由 PPS_OUT_PIN / SYNC_IN_PIN 配置，-1 关闭；对应 SERIALx_PROTOCOL 必须为 -1
    pps_gpio = plane.g2.pps_out_pin.get();
    sync_gpio = plane.g2.sync_in_pin.get();
#if !AP_SIM_ENABLED
    if (pps_gpio >= 0) {
        hal.gpio->pinMode(pps_gpio, HAL_GPIO_OUTPUT);
        hal.gpio->write(pps_gpio, 0);
    }
    if (sync_gpio >= 0) {
        hal.gpio->pinMode(sync_gpio, HAL_GPIO_INPUT);
        hal.gpio->attach_interrupt(sync_gpio,
                                   FUNCTOR_BIND_MEMBER(&Uart::sync_irq, void, uint8_t, bool, uint32_t),
                                   AP_HAL::GPIO::INTERRUPT_RISING);
    }
#endif
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&Uart::pps_tick, void));

    gcs().send_text(MAV_SEVERITY_WARNING, "TIMESYNC PPS_OUT_PIN=%d SYNC_IN_PIN=%d",
                    (int)pps_gpio, (int)sync_gpio);

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
    update_status();
    check_time_scale();
    bool auto_switch0 = plane.uattack.is_active_cam();
    bool auto_switch1 = plane.control_mode == &plane.mode_auto;
    bool auto_switch2 = (status.valid_diaocang) && (status.target_diaocang>0);
    bool auto_switch3 = status.valid_huokong_ok;

    bool auto_switch_flight = auto_switch0 && auto_switch1;
    bool auto_switch_target = auto_switch2 || auto_switch3;

    if (auto_switch_flight && auto_switch_target) {
        plane.set_mode(plane.mode_attack_loc, ModeReason::AUTO_ATTACK);
    }
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
    // 20ms 时统中断只置同步标志，打包发送仍在 100Hz 任务里做
    if (sync_pending) {
        sync_pending = false;
        request_send();
    }
    send_hangdong();
}

void Uart::update_status()
{
    if (millis() < 3000) {return;}
    // update status of diaocang
    if (millis() - status.last_diaocang_ms > 2000) {
        if (status.valid_diaocang) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Diaocang Lost");
        }
        status.valid_diaocang = false;
        status.target_diaocang = 0;
    } else {
        if (!status.valid_diaocang) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Diaocang Get");
        }
        status.valid_diaocang = true;
    }

    // update status of huokong
    if (millis() - status.last_huokong_ms > 2000) {
        if (status.valid_huokong) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Huokong Lost");
        }
        status.valid_huokong = false;
    } else {
        if (!status.valid_huokong) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Huokong Get");
        }
        status.valid_huokong = true;
    }

    // update status of huokong aim
    if (millis() - status.last_huokong_ok_ms > 2000) {
        if (status.valid_huokong_ok) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Huokong BAD");
        }
        status.valid_huokong_ok = false;
    } else {
        if (!status.valid_huokong_ok) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Huokong AIM");
        }
        status.valid_huokong_ok = true;
    }

    // update status of shuangmu
    if (millis() - status.last_shuangmu_ms > 2000) {
        if (status.valid_shuangmu) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Shuangmu Lost");
        }
        status.valid_shuangmu = false;
    } else {
        if (!status.valid_shuangmu) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Shuangmu Get");
        }
        status.valid_shuangmu = true;
    }

    // update status of waibu
    if (millis() - status.last_waibu_ms > 2000) {
        if (status.valid_waibu) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Waibu Lost");
        }
        status.valid_waibu = false;
    } else {
        if (!status.valid_waibu) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG Waibu Get");
        }
        status.valid_waibu = true;
    }
}

void Uart::handle_info_diaocang()
{
    status.target_diaocang = 0;
    for (uint8_t i_target = 1; i_target <=6; i_target++) {
        if (uart_msg_diaocang_cast._msg_1.content.msg.status_1 & (uint16_t)(1<<(6+i_target)))
        // 目标i跟踪好
        {
            status.target_diaocang++;

            uint16_t target_dist = uart_msg_diaocang_cast._msg_1.content.msg.target_dist;
            if (target_dist > 0) {
                plane.uattack.set_target_dist((float)target_dist);
            }

            float gimbal_yaw = uart_msg_diaocang_cast.get_angle(uart_msg_diaocang_cast._msg_1.content.msg.gimbal_yaw);
            float gimbal_pitch = uart_msg_diaocang_cast.get_angle(uart_msg_diaocang_cast._msg_1.content.msg.gimbal_pitch);
            float target_yaw = (float)(uart_msg_diaocang_cast._msg_1.content.msg.target[i_target-1].yaw) * 0.001f;
            float target_pitch = (float)(uart_msg_diaocang_cast._msg_1.content.msg.target[i_target-1].pitch) * 0.001f;

            Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
            Matrix3f tmp_target_cam_m;
            tmp_target_cam_m.from_euler(0.0f, radians(target_pitch), radians(target_yaw));
            Matrix3f tmp_cam_frame_m;
            tmp_cam_frame_m.from_euler(radians(0.0f), radians(gimbal_pitch), radians(gimbal_yaw));
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
                if (!status.valid_huokong_ok) {
                    // plane.uattack._Target_ptr_cam_FP847->handle_info(wrap_360(gimbal_yaw + target_yaw), wrap_180(gimbal_pitch + target_pitch));
                    plane.uattack._Target_ptr_cam_FP847->handle_info(angle_yaw, angle_pitch);
                }
            }
            break;
        }
    }

    status.last_diaocang_ms = millis();
    status.count_diaocang++;
}

void Uart::handle_info_huokong()
{
    uint8_t status_ok = uart_msg_huokong_cast._msg_1.content.msg.status_1 & (uint16_t)(1<<1);
    if (status_ok)
    // 诸元有效
    {
        float angle_pitch = uart_msg_huokong_cast.get_angle(uart_msg_huokong_cast._msg_1.content.msg.aim_pitch);
        float angle_yaw =   uart_msg_huokong_cast.get_angle(uart_msg_huokong_cast._msg_1.content.msg.aim_yaw);

        if (plane.uattack._Target_ptr_cam_FP847 != nullptr) {
            // plane.uattack._Target_ptr_cam_FP847->handle_info(wrap_360(gimbal_yaw + target_yaw), wrap_180(gimbal_pitch + target_pitch));
            plane.uattack._Target_ptr_cam_FP847->handle_info(angle_yaw, angle_pitch);
        }
        
        status.last_huokong_ok_ms = millis();
    }

    cali_last_ms = millis();
    cali_last_ms_global = get_time_ms(uart_msg_huokong_cast._msg_1.content.msg.time);

    status.last_huokong_ms = millis();
    status.count_huokong++;
}

void Uart::handle_info_shuangmu()
{
    status.last_shuangmu_ms = millis();
    status.count_shuangmu++;
}

void Uart::handle_info_waibu()
{
    status.last_waibu_ms = millis();
    status.count_waibu++;
    Location base_loc;
    // 协议 Loc_t 的经纬度是 1e-6 度，Location 内部是 1e-7 度
    base_loc.lat = (int32_t)uart_msg_waibu_single._msg_1.content.msg.base_loc.lat * 10;
    base_loc.lng = (int32_t)uart_msg_waibu_single._msg_1.content.msg.base_loc.lng * 10;
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
    // 20ms 周期作为保底；收到触发指令（need_send）时立即发送
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    // 有 20ms 时统时只在时统沿发送；时统丢失（>100ms 无沿）才回落到内部 20ms 周期
    const bool periodic = !sync_valid() && ((now - last_ms) >= 20);
    const bool triggered = uart_msg_hangdong_cast._msg_1.need_send;
    if (!periodic && !triggered) {
        return;
    }

    if (get_port() == nullptr) {return;}

    last_ms = now;

    set_tims_ms(uart_msg_hangdong_cast._msg_1.content.msg.time);
    uart_msg_hangdong_cast._msg_1.content.msg.status_1 = 0b0010111;
    uart_msg_hangdong_cast._msg_1.content.msg.error_1 = 0;
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D && AP::gps().ground_speed() > 3.f) {
        uart_msg_hangdong_cast._msg_1.content.msg.bearing = (uint32_t)(wrap_360(AP::gps().ground_course()) * 10000.f);
    } else {
        uart_msg_hangdong_cast._msg_1.content.msg.bearing = (int32_t)(wrap_360(degrees(AP::ahrs().get_yaw())) * 10000.f);
        // gcs().send_text(MAV_SEVERITY_INFO, "msg.bearing %x", uart_msg_hangdong_cast._msg_1.content.msg.bearing);
        // gcs().send_text(MAV_SEVERITY_INFO, "msg.bearing %d", int(uart_msg_hangdong_cast._msg_1.content.msg.bearing));
    }
    uart_msg_hangdong_cast._msg_1.content.msg.yaw = (int32_t)(degrees(AP::ahrs().get_yaw()) * 10000.f);
    uart_msg_hangdong_cast._msg_1.content.msg.pitch = (int32_t)(degrees(AP::ahrs().get_pitch()) * 10000.f);
    uart_msg_hangdong_cast._msg_1.content.msg.roll = (int32_t)(degrees(AP::ahrs().get_roll()) * 10000.f);

    Matrix3f tmp_body_yaw_m;
    tmp_body_yaw_m.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), 0.0f);
    Vector3f earth_rate = tmp_body_yaw_m * AP::ahrs().get_gyro();
    uart_msg_hangdong_cast._msg_1.content.msg.yaw_rate = (int16_t)(wrap_360(degrees(earth_rate.z)) * 100.f);
    uart_msg_hangdong_cast._msg_1.content.msg.pitch_rate = (int16_t)(degrees(earth_rate.y) * 100.f);
    uart_msg_hangdong_cast._msg_1.content.msg.roll_rate = (int16_t)(degrees(earth_rate.x) * 100.f);
    if (plane.airspeed.get_airspeed() > 0) {
        uart_msg_hangdong_cast._msg_1.content.msg.airspeed = (uint16_t)(plane.airspeed.get_airspeed());
    } else {
        uart_msg_hangdong_cast._msg_1.content.msg.airspeed = 0;
    }
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        uart_msg_hangdong_cast._msg_1.content.msg.status_1 |= 0b1000;
        uart_msg_hangdong_cast._msg_1.content.msg.ground_speed = (uint16_t)(AP::gps().ground_speed());
        uart_msg_hangdong_cast._msg_1.content.msg.ground_course = (uint32_t)(wrap_360(AP::gps().ground_course()) * 10000.f);
        uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant = 0;
        if (plane.current_loc.lat > 0) {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lat = plane.current_loc.lat/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant |= 0x01;
        } else {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lat = -plane.current_loc.lat/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant |= 0x02;
        }
        if (plane.current_loc.lng > 0) {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lng = plane.current_loc.lng/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant |= 0x10;
        } else {
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.lng = -plane.current_loc.lng/10;
            uart_msg_hangdong_cast._msg_1.content.msg.current_loc.quadrant |= 0x20;
        }
        int32_t tmp_alt_cm = 0;
        uart_msg_hangdong_cast._msg_1.content.msg.current_loc.abs_alt = 0;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt_cm)) {
            if (tmp_alt_cm > 0) {
                uart_msg_hangdong_cast._msg_1.content.msg.current_loc.abs_alt = tmp_alt_cm/100;
            }
        }

        uint8_t year_out = 0;
        uint8_t month_out = 0;
        uint8_t day_out = 0;
        uint8_t hour_out = 0;
        uint8_t minute_out = 0;
        uint8_t second_out = 0;
        get_Time(year_out, month_out, day_out, hour_out, minute_out, second_out);

        uart_msg_hangdong_cast._msg_1.content.msg.year = year_out + 2000;
        uart_msg_hangdong_cast._msg_1.content.msg.month = month_out;
        uart_msg_hangdong_cast._msg_1.content.msg.day = day_out;
        uart_msg_hangdong_cast._msg_1.content.msg.hour = hour_out;
        uart_msg_hangdong_cast._msg_1.content.msg.minute = minute_out;
        uart_msg_hangdong_cast._msg_1.content.msg.second = second_out;
    }

    uart_msg_hangdong_cast._msg_1.content.msg.yaw_corr = uart_msg_hangdong_cast.pack_angle(plane.g2.usr_calibration_yaw.get());
    uart_msg_hangdong_cast._msg_1.content.msg.pitch_corr = uart_msg_hangdong_cast.pack_angle(plane.g2.usr_calibration_pitch.get());

    uart_msg_hangdong_cast.make_sum();
    get_port()->write(uart_msg_hangdong_cast._msg_1.content.data, uart_msg_hangdong_cast._msg_1.content.msg.length);
    uart_msg_hangdong_cast._msg_1.need_send = false;
    // gcs().send_text(MAV_SEVERITY_INFO, "r: %0.1f, p: %0.1f, y: %0.1f", degrees(earth_rate.x), degrees(earth_rate.y), degrees(earth_rate.z));
}

void Uart::request_send()
{
    uart_msg_hangdong_cast._msg_1.need_send = true;
}

// 1kHz 定时器：1pps，脉宽 10ms
void Uart::pps_tick()
{
    if (pps_gpio < 0) {
        return;
    }
#if !AP_SIM_ENABLED
    hal.gpio->write(pps_gpio, pps_count < 10);
#endif
    if (++pps_count >= 1000) {
        pps_count = 0;
    }
}

// 20ms 时统上升沿中断
void Uart::sync_irq(uint8_t pin, bool state, uint32_t timestamp)
{
    if (!state) {
        return;
    }
    sync_last_us = timestamp;
    sync_count++;
    sync_pending = true;
}

bool Uart::sync_valid() const
{
    return (sync_gpio >= 0) && (sync_count > 0) && ((AP_HAL::micros() - sync_last_us) < 100000);
}

void Uart::check_time_scale()
{
    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        time_check_ms = 0;
        return;
    }

    const uint32_t now = millis();
    const uint32_t gps_ms = AP::gps().time_week_ms();

    if (time_check_ms == 0) {
        time_check_ms = now;
        time_gps_sec_last = gps_ms / 1000;
        time_edge_local = now;
        time_edge_count = 0;
        time_edge_err_sum = 0;
        return;
    }

    // GPS 时间随每次定位更新（5Hz），只在整秒跳变时取基准
    const uint32_t gps_sec = gps_ms / 1000;
    if (gps_sec == time_gps_sec_last) {
        return;
    }
    time_gps_sec_last = gps_sec;

    const uint32_t dt = now - time_edge_local;
    time_edge_local = now;
    if (time_edge_count > 0 && dt < 2000) {
        time_edge_err_sum += (int32_t)dt - 1000;
    }
    time_edge_count++;

    if (time_edge_count >= 60 && (time_edge_count % 60) == 0) {
        const int32_t mean = time_edge_err_sum / (int32_t)(time_edge_count - 1);
        const int32_t ppm = mean * 1000;
        gcs().send_text(MAV_SEVERITY_INFO, "UTIM n=%lu e=%ld ppm=%ld",
                        (unsigned long)time_edge_count, (long)mean, (long)ppm);
    }
}

void Uart::do_print()
{
    // 开机文本在 GCS 通道就绪前会丢，延迟到 5s 再补一次引脚配置
    if (!timesync_printed && millis() > 5000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "TIMESYNC PPS_OUT_PIN=%d SYNC_IN_PIN=%d",
                        (int)pps_gpio, (int)sync_gpio);
        timesync_printed = true;
    }
    if (plane.g2.uart_print.get() > 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "|~~~~~~~ MSG ~~~~~~~|");
    }
    // put your 1Hz code here

    // 发送方向：航动发出去的位置+姿态（证明发了且值对）
    if (plane.g2.uart_print.get() & (1<<0)) { // 1
        const FD1_msg_hangdong_cast::MSG_Collection &m = uart_msg_hangdong_cast._msg_1.content.msg;
        float lat_deg = (float)m.current_loc.lat / 1e6f;
        if (m.current_loc.quadrant & 0x02) { lat_deg = -lat_deg; }
        float lng_deg = (float)m.current_loc.lng / 1e6f;
        if (m.current_loc.quadrant & 0x20) { lng_deg = -lng_deg; }
        gcs().send_text(MAV_SEVERITY_WARNING, "HD send lat=%.6f lng=%.6f alt=%d yaw=%.1f pit=%.1f rol=%.1f gspd=%d",
                        lat_deg, lng_deg, (int)m.current_loc.abs_alt,
                        (float)m.yaw/10000.0f, (float)m.pitch/10000.0f, (float)m.roll/10000.0f, (int)m.ground_speed);
    }

    // 接收方向：火控诸元
    if (plane.g2.uart_print.get() & (1<<1)) { // 2
        const FD1_msg_huokong_cast::MSG_Collection &m = uart_msg_huokong_cast._msg_1.content.msg;
        gcs().send_text(MAV_SEVERITY_WARNING, "HK recv [%d] st=%d aim=(%.1f,%.1f) tlat=%lu tlng=%lu",
                        status.count_huokong, m.status_1,
                        uart_msg_huokong_cast.get_angle(m.aim_yaw), uart_msg_huokong_cast.get_angle(m.aim_pitch),
                        (unsigned long)m.target_loc.lat, (unsigned long)m.target_loc.lng);
    }

    // 接收方向：吊舱
    if (plane.g2.uart_print.get() & (1<<2)) { // 4
        const FD1_msg_diaocang_cast::MSG_Collection &m = uart_msg_diaocang_cast._msg_1.content.msg;
        gcs().send_text(MAV_SEVERITY_WARNING, "DC recv [%d] st=%d dist=%d gyaw=%.1f gpit=%.1f",
                        status.count_diaocang, m.status_1, (int)m.target_dist,
                        uart_msg_diaocang_cast.get_angle(m.gimbal_yaw), uart_msg_diaocang_cast.get_angle(m.gimbal_pitch));
    }

    // 接收方向：双目
    if (plane.g2.uart_print.get() & (1<<3)) { // 8
        const FD1_msg_shuangmu_cast::MSG_Collection &m = uart_msg_shuangmu_cast._msg_1.content.msg;
        gcs().send_text(MAV_SEVERITY_WARNING, "SH recv [%d] st=%d dist=%d tyaw=%.2f tpit=%.2f",
                        status.count_shuangmu, m.status_1, (int)m.target_dist,
                        (float)m.target.yaw*0.001f, (float)m.target.pitch*0.001f);
    }

    // 接收方向：任务/地面终端
    if (plane.g2.uart_print.get() & (1<<4)) { // 16
        const FD1_msg_waibu_single::MSG_Collection &m = uart_msg_waibu_single._msg_1.content.msg;
        const FD1_msg_waibu_single::Target_base_t &t = m.target[0];
        gcs().send_text(MAV_SEVERITY_WARNING, "WA recv [%d] ctl=%d blat=%lu blng=%lu R=%.1f A=%.2f E=%.2f",
                        status.count_waibu, m.control_1,
                        (unsigned long)m.base_loc.lat, (unsigned long)m.base_loc.lng,
                        (float)t.R*0.1f, (float)t.A*0.01f, (float)t.E*0.01f);
    }

    status.count_huokong = 0;
    status.count_diaocang = 0;
    status.count_shuangmu = 0;
    status.count_waibu = 0;
}
