#include "FD_Target_DYT.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo FD_Target_DYT::var_info[] = {
    // TOUT 为基类 FD_Target_Cam 的共用成员，由本类注册（AP_Param 嵌套层数限制，见 FD_Target_Cam.h）
    AP_GROUPINFO("TOUT", 0, FD_Target_DYT, target_timeout, 2000),
    AP_GROUPINFO("CENT", 1, FD_Target_DYT, center_time, 3000),
    AP_GROUPINFO("TRKT", 2, FD_Target_DYT, track_time, 200),
    AP_GROUPEND
};

FD_Target_DYT::FD_Target_DYT() :
    last_center_ms(0),
    last_track_ms(0),
    last_cancel_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_DYT::init()
{
    _last_ms = 0;
    _valid = false;
    set_type(0);

    if (!init_port()) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "UAttack DYT UART initialized");
    return true;
}

void FD_Target_DYT::update()
{
    if (get_port() == nullptr) {
        return;
    }

    while (get_port()->available() > 0) {
        const int16_t read_value = get_port()->read();
        if (read_value < 0) {
            break;
        }
        uart_msg_DYT_telem.parse((uint8_t)read_value);
        if (!uart_msg_DYT_telem._msg_1.updated) {
            continue;
        }

        if ((uart_msg_DYT_telem._msg_1.content.msg.status_1 & 0x04U) != 0U) {
            const float gimbal_yaw_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw * 0.01f;
            const float gimbal_pitch_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch * 0.01f;
            const float target_yaw_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.target_yaw * 0.05f;
            const float target_pitch_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.target_pitch * 0.05f;

            const Vector3f target_axis(1.0f, 0.0f, 0.0f);
            Matrix3f target_from_camera;
            target_from_camera.from_euler(0.0f, radians(target_pitch_deg), radians(target_yaw_deg));
            Matrix3f camera_from_sensor;
            camera_from_sensor.from_euler(0.0f, radians(gimbal_pitch_deg), radians(gimbal_yaw_deg));
            const Vector3f camera_los = camera_from_sensor * target_from_camera * target_axis;

            const float yaw_deg = degrees(atan2f(camera_los.y, camera_los.x));
            const float pitch_deg = degrees(atan2f(-camera_los.z, camera_los.xy().length()));
            handle_info(yaw_deg, pitch_deg);
            update_log();
        }
        uart_msg_DYT_telem._msg_1.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    update_timeout(now_ms);

    if (!_valid) {
        if (now_ms - last_center_ms > (uint32_t)center_time.get()) {
            uart_msg_DYT_control.pack_center();
            write_bytes(uart_msg_DYT_control._msg_1.content.data,
                        sizeof(uart_msg_DYT_control._msg_1.content.data));
            uart_msg_DYT_control.pack_open_recognition();
            write_bytes(uart_msg_DYT_control._msg_1.content.data,
                        sizeof(uart_msg_DYT_control._msg_1.content.data));
            last_center_ms = now_ms;
        }
        if ((now_ms - last_track_ms > (uint32_t)track_time.get()) &&
            (now_ms - last_cancel_ms > 2000U)) {
            uart_msg_DYT_control.pack_track();
            write_bytes(uart_msg_DYT_control._msg_1.content.data,
                        sizeof(uart_msg_DYT_control._msg_1.content.data));
            last_track_ms = now_ms;
        }
        return;
    }

    const float gimbal_yaw_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw * 0.01f;
    const float gimbal_pitch_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch * 0.01f;
    const float target_yaw_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.target_yaw * 0.05f;
    const float target_pitch_deg = (float)uart_msg_DYT_telem._msg_1.content.msg.target_pitch * 0.05f;
    if ((now_ms - last_cancel_ms > 500U) &&
        ((fabsf(gimbal_yaw_deg) > 120.0f) ||
         (fabsf(gimbal_pitch_deg) > 40.0f) ||
         ((fabsf(target_yaw_deg) > 8.0f) && (fabsf(target_pitch_deg) > 5.0f)))) {
        uart_msg_DYT_control.pack_cancel();
        write_bytes(uart_msg_DYT_control._msg_1.content.data,
                    sizeof(uart_msg_DYT_control._msg_1.content.data));
        _valid = false;
        last_cancel_ms = now_ms;
    }
}

void FD_Target_DYT::update_log()
{
    AP::logger().WriteStreaming(
        "DYT1",
        "TimeUS,rr,pr,yr,gy,gp,ty,tp",
        "s-------",
        "F-------",
        "Qfffffff",
        AP_HAL::micros64(),
        (float)uart_msg_DYT_telem._msg_1.content.msg.roll_rate * 0.01f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.pitch_rate * 0.01f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.yaw_rate * 0.01f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw * 0.01f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch * 0.01f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.target_yaw * 0.05f,
        (float)uart_msg_DYT_telem._msg_1.content.msg.target_pitch * 0.05f);
}
