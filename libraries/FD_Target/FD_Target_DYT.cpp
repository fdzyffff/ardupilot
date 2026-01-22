#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_DYT::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_DYT, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  1, FD_Target_DYT, cam_width,             360),
    AP_GROUPINFO("PIX_H",  2, FD_Target_DYT, cam_height,            360),
    AP_GROUPINFO("ANG_X",  3, FD_Target_DYT, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  4, FD_Target_DYT, cam_angle_y,           60.0f),

    AP_GROUPEND
};

FD_Target_DYT::FD_Target_DYT()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_DYT::init() {
    _last_ms = 0;
    _valid = false;
    set_type(0);
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NET, 0);
    if (_port == nullptr) {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "Uart DYT init");
    return true;
}

void FD_Target_DYT::update() {
    if (get_port() == nullptr) {return;}
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        uart_msg_DYT_telem.parse(temp);
        if (uart_msg_DYT_telem._msg_1.updated) {
                // gcs().send_text(MAV_SEVERITY_INFO, "uart_msg_DYT_telem._msg_1.content.msg.status_1: %x", uart_msg_DYT_telem._msg_1.content.msg.status_1);
            if (uart_msg_DYT_telem._msg_1.content.msg.status_1 & 0x04) {
                _last_ms = millis();

                float gimbal_yaw = (float)(uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw) * 0.01f;
                float gimbal_pitch = (float)(uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch) * 0.01f;
                float target_yaw = (float)(uart_msg_DYT_telem._msg_1.content.msg.target_yaw) * 0.05f;
                float target_pitch = (float)(uart_msg_DYT_telem._msg_1.content.msg.target_pitch) * 0.05f;

                Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
                Matrix3f tmp_target_cam_m;
                tmp_target_cam_m.from_euler(0.0f, radians(target_pitch), radians(target_yaw));
                Matrix3f tmp_cam_frame_m;
                tmp_cam_frame_m.from_euler(radians(0.0f), radians(gimbal_pitch), radians(gimbal_yaw));
                Matrix3f tmp_target_frame_m = tmp_cam_frame_m*tmp_target_cam_m;
                Vector3f ef_unit = tmp_target_frame_m*target_unit;

                float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));
                float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));

                handle_info(angle_yaw, angle_pitch);

                // gcs().send_text(MAV_SEVERITY_INFO, "angle_yaw: %f, angle_pitch: %f", angle_yaw, angle_pitch);
            }
            uart_msg_DYT_telem._msg_1.updated = false;
        }
    }

    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((target_timeout > 0) && (tnow - _last_ms > (uint32_t)target_timeout)) {
        // if (_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "valid %ld|%ld", tnow, _last_ms);
        // }
        _valid = false;
    }

    if (!_valid) {
        if (millis() - last_center_ms > 2000) {
            uart_msg_DYT_control.pack_center();
            last_center_ms = millis();
            get_port()->write(uart_msg_DYT_control._msg_1.content.data, sizeof(uart_msg_DYT_control._msg_1.content.data));
        }

        if (millis() - last_track_ms > 200) {
            uart_msg_DYT_control.pack_track();
            last_track_ms = millis();
            get_port()->write(uart_msg_DYT_control._msg_1.content.data, sizeof(uart_msg_DYT_control._msg_1.content.data));
        }
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;

                // float gimbal_yaw = (float)(uart_msg_DYT_telem._msg_1.content.msg.gimbal_yaw) * 0.01f;
                // float gimbal_pitch = (float)(uart_msg_DYT_telem._msg_1.content.msg.gimbal_pitch) * 0.01f;
                // float target_yaw = (float)(uart_msg_DYT_telem._msg_1.content.msg.target_yaw) * 0.05f;
                // float target_pitch = (float)(uart_msg_DYT_telem._msg_1.content.msg.target_pitch) * 0.05f;
                // gcs().send_text(MAV_SEVERITY_INFO, "gimbal_yaw: %f, gimbal_pitch: %f", gimbal_yaw, gimbal_pitch);
                // gcs().send_text(MAV_SEVERITY_INFO, "target_yaw: %f, target_pitch: %f", target_yaw, target_pitch);
    }
}

float FD_Target_DYT::cal_frame_angle(float pixel, float angle, float x_in)
{
    // pixel, eg: 1080
    // angle, eg: 54°
    // x_in, eg: 540
    // ret, eg: 0°
    pixel = constrain_float(pixel, 100.0f, 8000.f);
    angle = constrain_float(radians(angle), radians(10.0f), radians(150.0f));
    x_in = constrain_float(x_in, -pixel, pixel);
    float ret = atanf(2.0f*x_in/pixel*tanf(angle*0.5f));
    return degrees(ret);
}

void FD_Target_DYT::handle_info_test(float p1, float p2) {
    handle_info(p1, p2);
    // FD_K230_TARGET &tmp_msg = FD_K230_ptr->get_msg_cam_target();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    // tmp_msg._msg_1.content.msg.status = 1;
}

void FD_Target_DYT::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                gcs().send_text(MAV_SEVERITY_WARNING, "Target K230 Test");
                handle_info_test(packet.param1, packet.param2);
                break;
            default:
                break;
        }
    }

}
