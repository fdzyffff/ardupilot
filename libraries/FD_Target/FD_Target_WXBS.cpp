#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_WXBS::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_WXBS, target_timeout,        2000),
    AP_GROUPINFO("PIX_W",  1, FD_Target_WXBS, cam_width,             360),
    AP_GROUPINFO("PIX_H",  2, FD_Target_WXBS, cam_height,            360),
    AP_GROUPINFO("ANG_X",  3, FD_Target_WXBS, cam_angle_x,           60.0f),
    AP_GROUPINFO("ANG_Y",  4, FD_Target_WXBS, cam_angle_y,           60.0f),

    AP_GROUPEND
};

FD_Target_WXBS::FD_Target_WXBS()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_WXBS::init() {
    return true;
}

void FD_Target_WXBS::update() {
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
}

void FD_Target_WXBS::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_WXBS_ATTACK_INFO) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_wxbs_attack_info_t packet;
        mavlink_msg_wxbs_attack_info_decode(&msg, &packet);
        if (packet.have_target) {
            _last_ms = millis();
            float theta1 = ((float)packet.yaw_angle_cd) * 0.01f; // x-axis, degree
            float theta2 = ((float)packet.pitch_angle_cd) * 0.01f; // y-axis, degree

            Vector3f tmp = Vector3f(1.f, tanf(radians(theta1)), -tanf(radians(theta2)));
            float p1 =  degrees(atanf(tmp.y/tmp.x));
            float p2 = -degrees(atanf(tmp.z/tmp.xy().length()));
            handle_info(p1, p2);
        }
    }
}

void FD_Target_WXBS::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
