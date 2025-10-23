#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_QD::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_QD, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  1, FD_Target_QD, cam_width,             360),
    AP_GROUPINFO("PIX_H",  2, FD_Target_QD, cam_height,            360),
    AP_GROUPINFO("ANG_X",  3, FD_Target_QD, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  4, FD_Target_QD, cam_angle_y,           60.0f),
    AP_GROUPINFO("DEBUG",  5, FD_Target_QD, cam_debug,             0),

    AP_GROUPEND
};

FD_Target_QD::FD_Target_QD()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_QD::init() {
    _last_ms = 0;
    _valid = false;
    FD_QD_ptr = new FD_QD(AP_SerialManager::SerialProtocol_CAM);
    FD_QD_ptr->init();
    FD_QD_ptr->get_msg_QD_S11().set_enable();
    return FD_QD_ptr->initialized();
}

void FD_Target_QD::update() {
    static uint32_t last_update_ms = millis();

    static uint32_t last_print_ms = millis();
    bool do_print = false;
    if (millis() - last_print_ms > 1000) {
        if (cam_debug.get()) {
            do_print = true;
        }
        last_print_ms = millis();
    }

    FD_QD_ptr->read();
    FD_msg_QD_S11 &tmp_msg = FD_QD_ptr->get_msg_QD_S11();
    if (tmp_msg._msg_1.updated) {

        if (tmp_msg._msg_1.content.msg.track_status == 0x02) {
            _last_ms = millis();
            float theta1 =  cal_frame_angle_left_up(cam_width.get(), cam_angle_x.get(), tmp_msg._msg_1.content.msg.target_x); // x-axis, degree
            float theta2 = -cal_frame_angle_left_up(cam_height.get(), cam_angle_y.get(), tmp_msg._msg_1.content.msg.target_y); // y-axis, degree

            if (do_print) {
                gcs().send_text(MAV_SEVERITY_INFO, "theta (%0.1f, %0.1f)", theta1, theta2);
            }

            Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
            float tgt_p1 = degrees(atanf(tmp.y/tmp.x));
            float tgt_p2 = degrees(atanf(tmp.z/tmp.xy().length()));

            if (do_print) {
                gcs().send_text(MAV_SEVERITY_INFO, "tgt (%0.1f, %0.1f)", tgt_p1, tgt_p2);
            }

            Vector3f cam_unit = Vector3f(1.0f, 0.0f, 0.0f);
            Matrix3f tmp_target_cam_m;
            tmp_target_cam_m.from_euler(0.0f, radians(tgt_p2), radians(tgt_p1));

            float LSB = 360.f/65536.f;
            float cam_yaw = (float)(tmp_msg._msg_1.content.msg.cam_yaw)*LSB;
            float cam_pitch = (float)(tmp_msg._msg_1.content.msg.cam_pitch)*LSB;

            if (do_print) {
                gcs().send_text(MAV_SEVERITY_INFO, "cam (%0.1f, %0.1f)", cam_yaw, cam_pitch);
            }

            Matrix3f tmp_cam_body_m;
            tmp_cam_body_m.from_euler(0.0f, radians(cam_pitch), radians(cam_yaw));
            Vector3f bef_cam_unit = tmp_cam_body_m*tmp_target_cam_m*cam_unit;

            float p1 = degrees(atanf(bef_cam_unit.y/bef_cam_unit.x));
            float p2 = degrees(atanf(bef_cam_unit.z/bef_cam_unit.xy().length()));

            if (do_print) {
                gcs().send_text(MAV_SEVERITY_INFO, "theta (%0.1f, %0.1f)", p1, p2);
            }

            handle_info(p1, p2);
        }

        tmp_msg._msg_1.updated = false;   
    }
    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((target_timeout > 0) && (tnow - _last_ms > (uint32_t)target_timeout)) {
        // if (_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "valid %ld|%ld", tnow, _last_ms);
        // }
        _valid = false;
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

float FD_Target_QD::cal_frame_angle(float pixel, float angle, float x_in)
{
    // pixel, eg: 1080
    // angle, eg: 54°
    // x_in, eg: 540
    // ret, eg: 0°
    pixel = constrain_float(pixel, 100.0f, 8000.f);
    angle = constrain_float(radians(angle), radians(10.0f), radians(150.0f));
    x_in = constrain_float(x_in, -pixel*0.5f, pixel*0.5f);
    float ret = atanf(2.0f*(x_in)/pixel*tanf(angle*0.5f));
    return degrees(ret);
}

float FD_Target_QD::cal_frame_angle_left_up(float pixel, float angle, float x_in)
{
    // pixel, eg: 1080
    // angle, eg: 54°
    // x_in, eg: 540
    // ret, eg: 0°
    pixel = constrain_float(pixel, 100.0f, 8000.f);
    angle = constrain_float(radians(angle), radians(10.0f), radians(150.0f));
    x_in = constrain_float(x_in, 0.f, pixel);
    float ret = atanf(2.0f*(x_in-pixel*0.5f)/pixel*tanf(angle*0.5f));
    return degrees(ret);
}

void FD_Target_QD::handle_info_test(float p1, float p2) {
    handle_info(p1, p2);
    // FD_QD_TARGET &tmp_msg = FD_QD_ptr->get_msg_cam_target();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    // tmp_msg._msg_1.content.msg.status = 1;
}

void FD_Target_QD::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                gcs().send_text(MAV_SEVERITY_WARNING, "Target QD Test");
                handle_info_test(packet.param1, packet.param2);
                break;
            default:
                break;
        }
    }

}
