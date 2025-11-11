#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_RK3588::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_RK3588, target_timeout,        2000),
    AP_GROUPINFO("ANG_X",  1, FD_Target_RK3588, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  2, FD_Target_RK3588, cam_angle_y,           60.0f),

    AP_GROUPEND
};

FD_Target_RK3588::FD_Target_RK3588()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_RK3588::init() {
    _last_ms = 0;
    _valid = false;
    FD_RK3588_ptr = new FD_RK3588(AP_SerialManager::SerialProtocol_CAM);
    FD_RK3588_ptr->init();
    FD_RK3588_ptr->get_msg_RK3588().set_enable();
    return FD_RK3588_ptr->initialized();
}

void FD_Target_RK3588::update() {
    static uint32_t last_update_ms = millis();

    FD_RK3588_ptr->read();
    FD_msg_RK3588 &tmp_msg = FD_RK3588_ptr->get_msg_RK3588();
    if (tmp_msg._msg_1.updated) {

        // if (tmp_msg._msg_1.content.msg.tag_cl > 0.5f) {
        _last_ms = millis();
        float theta1 =  cal_frame_angle(cam_angle_x.get(), tmp_msg._msg_1.content.msg.tag_x); // x-axis, degree
        float theta2 = -cal_frame_angle(cam_angle_y.get(), tmp_msg._msg_1.content.msg.tag_y); // y-axis, degree //change +/-

        // handle_info(tmp_msg._msg_1.content.msg.tag_x*100.f, tmp_msg._msg_1.content.msg.tag_y*100.f);
        Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
        float p1 =  degrees(atanf(tmp.y/tmp.x));
        float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
        handle_info(p1, p2);
        // }

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

float FD_Target_RK3588::cal_frame_angle(float angle, float x_in)
{
    // angle, eg: 54°
    // x_in, eg: -0.5
    // ret, eg: 0°
    angle = constrain_float(radians(angle), radians(10.0f), radians(150.0f));
    x_in = constrain_float(x_in, -1.0f, 1.0f);
    float ret = atanf(x_in*tanf(angle*0.5f));
    return degrees(ret);
}

void FD_Target_RK3588::handle_info_test(float p1, float p2) {
    handle_info(p1, p2);
    // FD_RK3588_TARGET &tmp_msg = FD_RK3588_ptr->get_msg_cam_target();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    // tmp_msg._msg_1.content.msg.status = 1;
}

void FD_Target_RK3588::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                gcs().send_text(MAV_SEVERITY_WARNING, "Target RK3588 Test");
                handle_info_test(packet.param1, packet.param2);
                break;
            default:
                break;
        }
    }

}
