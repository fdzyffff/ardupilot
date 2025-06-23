#include "FD_Target.h"
#include <AP_Mount/AP_Mount.h>

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_Topotek::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_Topotek, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  1, FD_Target_Topotek, cam_width,             360),
    AP_GROUPINFO("PIX_H",  2, FD_Target_Topotek, cam_height,            360),
    AP_GROUPINFO("ANG_X",  3, FD_Target_Topotek, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  4, FD_Target_Topotek, cam_angle_y,           60.0f),

    AP_GROUPEND
};

FD_Target_Topotek::FD_Target_Topotek()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_Topotek::init() {
    return true;
}

void FD_Target_Topotek::update() {
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

void FD_Target_Topotek::test_cal()
{
    static uint32_t _last_test_ms = millis();
    if (millis() - _last_test_ms > 1000) {
        _last_test_ms = millis();

        _last_ms = millis();
        float theta1 =  cal_frame_angle(cam_width.get(), cam_angle_x.get(),  0.f); // x-axis, degree
        float theta2 =  cal_frame_angle(cam_height.get(), cam_angle_y.get(), 0.f); // y-axis, degree
        handle_raw_info(theta1, theta2);

        Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
        float p1 =  degrees(atanf(tmp.y/tmp.x));
        float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));

        float _roll = AP::ahrs().get_roll();
        float _pitch = AP::ahrs().get_pitch();
        _roll = radians(0.0f);
        _pitch = radians(10.0f);
        Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
        Matrix3f tmp_target_cam_m;
        tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
        Matrix3f tmp_cam_level_m;
        tmp_cam_level_m.from_euler(radians(0.0f), radians(0.0f), radians(90.f));

        Matrix3f tmp_level_body_m;
        tmp_level_body_m.from_euler(_roll, _pitch, 0.0f);
        tmp_level_body_m.transpose();
        Matrix3f tmp_target_earth_m = tmp_level_body_m*tmp_cam_level_m*tmp_target_cam_m;
        Vector3f bf_unit = tmp_target_earth_m*target_unit;

        float angle_yaw =   wrap_180(degrees(atan2f( bf_unit.y, bf_unit.x)));
        float angle_pitch = wrap_180(degrees(atan2f(-bf_unit.z, bf_unit.xy().length())));
        
        handle_info(angle_yaw, angle_pitch);
    }
}

void FD_Target_Topotek::handle_msg(const mavlink_message_t &msg)
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
                    float theta1 =  cal_frame_angle(cam_width.get(), cam_angle_x.get(), packet.param1); // x-axis, degree
                    float theta2 =  cal_frame_angle(cam_height.get(), cam_angle_y.get(), packet.param2); // y-axis, degree
                    // handle_raw_info(theta1, theta2);

                    Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
                    float p1 =  degrees(atanf(tmp.y/tmp.x));
                    float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
                    cal_and_handle(p1, p2);
                    // handle_info(theta1, theta2);
                }
                break;
            default:
                break;
        }
    }
}

void FD_Target_Topotek::cal_and_handle(float p1, float p2) 
{
    AP_Mount* mount = AP::mount();
    float cam_roll, cam_pitch, cam_bf_yaw;
    if (mount != nullptr) {
        mount->get_attitude_euler(0, cam_roll, cam_pitch, cam_bf_yaw);
    }
    // static uint32_t _last_info_ms = millis();
    // if (millis() - _last_info_ms > 1000) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "VV %d, %f, %f, %f", mount != nullptr, cam_roll, cam_pitch, cam_bf_yaw);
    //     _last_info_ms = millis();
    // }
    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_level_m;
    tmp_cam_level_m.from_euler(radians(cam_roll), radians(cam_pitch), radians(cam_bf_yaw));
    Matrix3f tmp_level_body_m;
    tmp_level_body_m.from_euler(_roll, _pitch, 0.0f);
    tmp_level_body_m.transpose();
    Matrix3f tmp_target_earth_m = tmp_level_body_m*tmp_cam_level_m*tmp_target_cam_m;
    Vector3f bf_unit = tmp_target_earth_m*target_unit;

    float angle_yaw =   wrap_180(degrees(atan2f( bf_unit.y, bf_unit.x)));
    float angle_pitch = wrap_180(degrees(atan2f(-bf_unit.z, bf_unit.xy().length())));
    
    handle_info(angle_yaw, angle_pitch);
}

float FD_Target_Topotek::cal_frame_angle(float pixel, float angle, float x_in)
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

void FD_Target_Topotek::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
