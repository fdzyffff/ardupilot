#include "FD_Target_HY.h"

#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo FD_Target_HY::var_info[] = {
    AP_GROUPINFO("TOUT", 0, FD_Target_HY, target_timeout, 2000),
    AP_GROUPINFO("RESX", 3, FD_Target_HY, res_x, 1920.0f),
    AP_GROUPINFO("RESY", 4, FD_Target_HY, res_y, 1080.0f),
    AP_GROUPINFO("FOVX", 5, FD_Target_HY, fov_x, 60.0f),
    AP_GROUPINFO("FOVY", 6, FD_Target_HY, fov_y, 45.0f),
    AP_GROUPINFO("AUTO", 11, FD_Target_HY, auto_lock_enable, 0),
    AP_GROUPINFO("ATIME", 12, FD_Target_HY, auto_lock_period_ms, 3000),
    AP_GROUPEND
};

FD_Target_HY::FD_Target_HY() :
    _port(nullptr)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_HY::init()
{
    _last_ms = 0;
    _valid = false;
    last_auto_lock_ms = 0;
    set_type(0);

    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NET, 0);
    if (_port == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UAttack HY UART unavailable");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "UAttack HY UART initialized");

    // 上电后自动配置导引头工作模式：交替发送 4.4.1/4.4.5 各5次。
    cfg_state = CfgState::RUNNING;
    cfg_detect_count = 0;
    cfg_autolock_count = 0;
    cfg_next_is_detect = true;
    cfg_last_ms = AP_HAL::millis();
    return true;
}

void FD_Target_HY::update()
{
    if (_port == nullptr) {
        return;
    }

    while (_port->available() > 0) {
        const int16_t read_value = _port->read();
        if (read_value < 0) {
            break;
        }
        uart_msg_HY_miss.parse((uint8_t)read_value);
        uart_msg_HY_ack.parse((uint8_t)read_value);
    }

    if (uart_msg_HY_miss._msg_1.updated) {
        handle_miss_msg();
        uart_msg_HY_miss._msg_1.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    handle_control_ack();
    run_startup_config(now_ms);

    if ((target_timeout.get() > 0) &&
        (now_ms - _last_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }

    const int32_t period_ms = auto_lock_period_ms.get();
    if ((auto_lock_enable.get() != 0) && (period_ms > 0) &&
        (now_ms - last_auto_lock_ms >= (uint32_t)period_ms)) {
        uart_msg_HY_control.pack_auto_lock(0x02, 0x00);
        send_control();
        last_auto_lock_ms = now_ms;
    }
}

void FD_Target_HY::handle_miss_msg()
{
    const FD1_msg_HY_miss::MSG_Collection &msg = uart_msg_HY_miss._msg_1.content.msg;

    const bool data_valid = (msg.status & 0x01U) != 0U;
    const bool algo_stopped = (msg.status & 0x02U) != 0U;
    const bool is_angle = (msg.status & 0x04U) != 0U;
    if (!data_valid || algo_stopped) {
        return;
    }

    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    if (is_angle) {
        yaw_deg = msg.offset_x.f;
        pitch_deg = msg.offset_y.f;
    } else if (!pixel_offset_to_spherical_deg((float)msg.offset_x.i,
                                               (float)msg.offset_y.i,
                                               yaw_deg, pitch_deg)) {
        return;
    }

    handle_info(yaw_deg, pitch_deg);
    update_log_miss((int16_t)(yaw_deg * 100.0f),
                    (int16_t)(pitch_deg * 100.0f),
                    msg.width, msg.height, msg.status);
}

bool FD_Target_HY::pixel_offset_to_spherical_deg(float offset_x, float offset_y,
                                                  float &yaw_deg, float &pitch_deg) const
{
    const float theta1 = cal_frame_angle(res_x.get(), fov_x.get(), offset_x);
    const float theta2 = cal_frame_angle(res_y.get(), fov_y.get(), offset_y);

    const Vector3f los_camera(1.0f, tanf(radians(theta1)), -tanf(radians(theta2)));
    yaw_deg = degrees(atanf(los_camera.y / los_camera.x));
    pitch_deg = -degrees(atanf(los_camera.z / los_camera.xy().length()));
    return true;
}

float FD_Target_HY::cal_frame_angle(float pixel, float angle, float x_in) const
{
    pixel = constrain_float(pixel, 100.0f, 8000.0f);
    angle = constrain_float(radians(angle), radians(10.0f), radians(80.0f));
    x_in = constrain_float(x_in, -pixel, pixel);
    const float ret = atanf(2.0f * x_in / pixel * tanf(angle * 0.5f));
    return degrees(ret);
}

void FD_Target_HY::send_control()
{
    if (_port == nullptr) {
        return;
    }
    _port->write(uart_msg_HY_control._msg_1.content.data,
                 uart_msg_HY_control._msg_1.length);
}

void FD_Target_HY::update_log_miss(int16_t yaw_cdeg, int16_t pitch_cdeg,
                                   uint16_t w, uint16_t h, uint8_t status)
{
    AP::logger().WriteStreaming(
        "HYM1",
        "TimeUS,St,Yaw,Pitch,W,H",
        "s-dd--",
        "F-----",
        "QBcccc",
        AP_HAL::micros64(),
        status,
        yaw_cdeg,
        pitch_cdeg,
        w,
        h);
}

void FD_Target_HY::run_startup_config(uint32_t now_ms)
{
    if (cfg_state != CfgState::RUNNING) {
        return;
    }
    if (now_ms - cfg_last_ms < 500U) {
        return;
    }
    cfg_last_ms = now_ms;

    if (cfg_next_is_detect) {
        uart_msg_HY_control.pack_detect(0x01);
        send_control();
        cfg_detect_count++;
        gcs().send_text(MAV_SEVERITY_INFO, "UAttack HY cfg 4.4.1 #%d sent",
                        cfg_detect_count);
    } else {
        uart_msg_HY_control.pack_auto_lock(0x02, 0x00);
        send_control();
        cfg_autolock_count++;
        gcs().send_text(MAV_SEVERITY_INFO, "UAttack HY cfg 4.4.5 #%d sent",
                        cfg_autolock_count);
    }
    cfg_next_is_detect = !cfg_next_is_detect;

    if ((cfg_detect_count >= 5U) && (cfg_autolock_count >= 5U)) {
        cfg_state = CfgState::DONE;
        gcs().send_text(MAV_SEVERITY_INFO, "UAttack HY cfg done");
    }
}

void FD_Target_HY::handle_control_ack()
{
    if (!uart_msg_HY_ack._msg_1.updated) {
        return;
    }
    const FD1_msg_HY_ack::MSG_Collection &msg = uart_msg_HY_ack._msg_1.content.msg;
    const uint8_t result = msg.payload[0];
    const uint8_t state = msg.payload[1];

    if (msg.cmd1 == 0x81) {  // 4.4.1 目标检测控制响应
        gcs().send_text(MAV_SEVERITY_INFO,
                        "UAttack HY cfg ack 4.4.1 result=%s state=%d",
                        (result == 0) ? "OK" : "FAIL", state);
    } else if (msg.cmd1 == 0x85) {  // 4.4.5 自动锁定响应
        gcs().send_text(MAV_SEVERITY_INFO,
                        "UAttack HY cfg ack 4.4.5 result=%s state=%d",
                        (result == 0) ? "OK" : "FAIL", state);
    }
    uart_msg_HY_ack._msg_1.updated = false;
}
