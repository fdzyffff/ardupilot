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
    }

    if (uart_msg_HY_miss._msg_1.updated) {
        handle_miss_msg();
        uart_msg_HY_miss._msg_1.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if ((target_timeout.get() > 0) &&
        (now_ms - _last_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }

    const int32_t period_ms = auto_lock_period_ms.get();
    if ((auto_lock_enable.get() != 0) && (period_ms > 0) &&
        (now_ms - last_auto_lock_ms >= (uint32_t)period_ms)) {
        uart_msg_HY_control.pack_auto_lock();
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
