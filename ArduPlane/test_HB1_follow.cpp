#include "Plane.h"

#define DELTAT 300

void Plane::test_HB1_init(){
    ;
}

void Plane::test_HB1_follow(uint8_t msg_id)
{
    HB1_test.status = msg_id;
    HB1_test.state = 0;
}

void Plane::test_HB1_follow_update(void)
{
    static uint32_t tlast = millis();
    uint32_t tnow = millis();
    if (HB1_test.status == 1) {
        if (tnow - tlast < DELTAT) {
            test_HB1_follow_target_update_1(float(tnow - tlast));
        } else {
            test_HB1_follow_target_reset();
        }
        tlast = tnow;
    }
    else if (HB1_test.status == 2) {
        if (tnow - tlast < DELTAT) {
            test_HB1_follow_target_update_2(float(tnow - tlast));
        } else {
            test_HB1_follow_target_reset();
        }
        tlast = tnow;
    }
}

void Plane::test_HB1_follow_target_update_1(float t_ms)
{
    const float x_len = 50000.0f;
    const float y_len = 100000.0f;
    float tmp_vel = 2000.0f;
    float tmp_length = tmp_vel * t_ms * 0.001f;
    Vector3f tmp_last_pos;
    if (!HB1_test.follow_loc.get_vector_from_origin_NEU(tmp_last_pos)) {
        return;
    }
    Vector3f tmp_target;
    float tmp_dir_ang = 0.0f;
    switch (HB1_test.state) {
        case 0:
            if (tmp_last_pos.x - tmp_length < -x_len) {
                tmp_target.x = -x_len;
                tmp_target.y = -x_len - (tmp_last_pos.x - tmp_length);
                HB1_test.state = 1;
                tmp_dir_ang = 90.0f;
            }
            tmp_target.x = tmp_last_pos.x - tmp_length;
            tmp_target.y = 0.0f;
            tmp_dir_ang = 180.0f;
            break;
        case 1:
            if (tmp_last_pos.y + tmp_length > y_len) {
                tmp_target.x = -x_len + tmp_length + tmp_last_pos.y - y_len;
                tmp_target.y = y_len;
                HB1_test.state = 2;
                tmp_dir_ang = 0.0f;
            }
            tmp_target.x = -x_len;
            tmp_target.y = tmp_last_pos.y + tmp_length;
            tmp_dir_ang = 90.0f;
            break;
        case 2:
            if (tmp_last_pos.x + tmp_length > 0.0f) {
                tmp_target.x = 0.0f;
                tmp_target.y = y_len - (tmp_last_pos.x + tmp_length);
                HB1_test.state = 3;
                tmp_dir_ang = 270.0f;
            }
            tmp_target.x = tmp_last_pos.x + tmp_length;
            tmp_target.y = y_len;
            tmp_dir_ang = 0.0f;
            break;
        case 3:
            if (tmp_last_pos.y - tmp_length < 0.0f) {
                tmp_target.x = (tmp_last_pos.y - tmp_length);
                tmp_target.y = 0.0f;
                HB1_test.state = 0;
                tmp_dir_ang = 180.0f;
            }
            tmp_target.x = 0.0f;
            tmp_target.y = tmp_last_pos.y - tmp_length;
            tmp_dir_ang = 270.0f;
            break;
        default:
            break;
    }
    if (HB1_test.state <= 3) {
        tmp_target.z = 10000.f;
        Location loc(tmp_target);
        HB1_test.follow_loc.lng = loc.lng;
        HB1_test.follow_loc.lat = loc.lat;
        HB1_test.follow_loc.set_alt_cm(tmp_target.z, Location::AltFrame::ABOVE_HOME);
    }

    // pack up msg
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = false;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
    tmp_msg._msg_1.content.msg.length = tmp_msg._msg_1.length-3;
        
    tmp_msg._msg_1.content.msg.console_type = 0;
    tmp_msg._msg_1.content.msg.remote_index = 6;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[0] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[1] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[2] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[3] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[4] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[5] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[6] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[7] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[8] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[9] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[10] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[11] = 0;
    tmp_msg._msg_1.content.msg.youshang_target_airspeed = 0;
    tmp_msg._msg_1.content.msg.youshang_target_orthdist = 0;
    tmp_msg._msg_1.content.msg.youshang_target_alt = 0;
    tmp_msg._msg_1.content.msg.apm_deltaX = 0;
    tmp_msg._msg_1.content.msg.apm_deltaY = 0;
    tmp_msg._msg_1.content.msg.apm_deltaZ = 0;
    tmp_msg._msg_1.content.msg.leader_lng = (int32_t)((double)HB1_test.follow_loc.lng*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_lat = (int32_t)((double)HB1_test.follow_loc.lat*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_alt = (int16_t)((float)HB1_test.follow_loc.alt*tmp_msg.SF_ALT);
    tmp_msg._msg_1.content.msg.leader_dir = (int16_t)(wrap_180(tmp_dir_ang)*tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.unused[0] = 0;
    tmp_msg._msg_1.content.msg.unused[1] = 0;
    tmp_msg._msg_1.content.msg.unused[2] = 0;
    tmp_msg._msg_1.content.msg.unused[3] = 0;
    tmp_msg._msg_1.content.msg.unused[4] = 0;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}


void Plane::test_HB1_follow_target_update_2(float t_ms)
{
    float tmp_vel = 2000.0f;
    float _radius = 10000.f;
    float _angular_vel = tmp_vel / _radius;
    float dt = t_ms * 0.001f;
    Vector3f tmp_last_pos;
    if (!HB1_test.follow_loc.get_vector_from_origin_NEU(tmp_last_pos)) {
        return;
    }
    Vector3f tmp_target;
    static float _angle = 0.0f;
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);

    tmp_target.x = 0.0f + _radius * cosf(-_angle);
    tmp_target.y = 0.0f - _radius * sinf(-_angle);

    if (HB1_test.state <= 3) {
        tmp_target.z = 10000.f;
        Location loc(tmp_target);
        HB1_test.follow_loc.lng = loc.lng;
        HB1_test.follow_loc.lat = loc.lat;
        HB1_test.follow_loc.set_alt_cm(tmp_target.z, Location::AltFrame::ABOVE_HOME);
    }
    float tmp_dir_ang = degrees(_angle) + 90.0f;

    // pack up msg
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = false;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
        
    tmp_msg._msg_1.content.msg.console_type = 0;
    tmp_msg._msg_1.content.msg.remote_index = 6;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[0] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[1] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[2] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[3] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[4] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[5] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[6] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[7] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[8] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[9] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[10] = 0;
    tmp_msg._msg_1.content.msg.remote_cmd.cmd_data[11] = 0;
    tmp_msg._msg_1.content.msg.youshang_target_airspeed = 0;
    tmp_msg._msg_1.content.msg.youshang_target_orthdist = 0;
    tmp_msg._msg_1.content.msg.youshang_target_alt = 0;
    tmp_msg._msg_1.content.msg.apm_deltaX = 0;
    tmp_msg._msg_1.content.msg.apm_deltaY = 0;
    tmp_msg._msg_1.content.msg.apm_deltaZ = 0;
    tmp_msg._msg_1.content.msg.leader_lng = (int32_t)((double)HB1_test.follow_loc.lng*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_lat = (int32_t)((double)HB1_test.follow_loc.lat*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_alt = (int16_t)((float)HB1_test.follow_loc.alt*tmp_msg.SF_ALT);
    tmp_msg._msg_1.content.msg.leader_dir = (int16_t)(wrap_180(tmp_dir_ang)*tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.unused[0] = 0;
    tmp_msg._msg_1.content.msg.unused[1] = 0;
    tmp_msg._msg_1.content.msg.unused[2] = 0;
    tmp_msg._msg_1.content.msg.unused[3] = 0;
    tmp_msg._msg_1.content.msg.unused[4] = 0;
    
    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_follow_target_reset(void)
{
    Vector3f tmp_target(0.0f, 0.0f, 10000.f);
    Location loc(tmp_target);
    HB1_test.follow_loc.lng = loc.lng;
    HB1_test.follow_loc.lat = loc.lat;
    HB1_test.follow_loc.set_alt_cm(tmp_target.z, Location::AltFrame::ABOVE_HOME);
    HB1_test.state = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "T follow reset");
}
