#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_Mav::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_Mav, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  1, FD_Target_Mav, cam_width,             360),
    AP_GROUPINFO("PIX_H",  2, FD_Target_Mav, cam_height,            360),
    AP_GROUPINFO("ANG_X",  3, FD_Target_Mav, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  4, FD_Target_Mav, cam_angle_y,           60.0f),

    AP_GROUPEND
};

FD_Target_Mav::FD_Target_Mav()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_Mav::init() {
    return true;
}

void FD_Target_Mav::update() {
    static uint32_t last_update_ms = millis();
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
    // for test purpose
    // test_cal();
}

void FD_Target_Mav::test_cal()
{
    static uint32_t _last_test_ms = millis();
    if (millis() - _last_test_ms > 1000) {
        _last_test_ms = millis();

        _last_ms = millis();
        float theta1 =  10.f; // x-axis, degree
        float theta2 =  -10.f; // y-axis, degree
        // gcs().send_text(MAV_SEVERITY_INFO, "t1 %f| t2 %f", theta1, theta2);

        Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
        float p1 =  degrees(atanf(tmp.y/tmp.x));
        float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
        handle_info(p1, p2);
    }
}

void FD_Target_Mav::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_5:
                {
                    _last_ms = millis();
                    // gcs().send_text(MAV_SEVERITY_INFO, "p1 %f| p2 %f", packet.param1, packet.param2);
                    float theta1 =  cal_frame_angle(cam_width.get(), cam_angle_x.get(), packet.param1); // x-axis, degree
                    float theta2 =  cal_frame_angle(cam_height.get(), cam_angle_y.get(), packet.param2); // y-axis, degree
                    // gcs().send_text(MAV_SEVERITY_INFO, "t1 %f| t2 %f", theta1, theta2);

                    Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
                    float p1 =  degrees(atanf(tmp.y/tmp.x));
                    float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
                    handle_info(p1, p2);
                    // handle_info(theta1, theta2);
                }
                break;
            default:
                break;
        }
    }
}

float FD_Target_Mav::cal_frame_angle(float pixel, float angle, float x_in)
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

void FD_Target_Mav::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
