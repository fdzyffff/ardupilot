#include "Plane.h"

#define DELTAT 300

void Plane::test_HB1_init(){
    ;
}

// first of all: use MAV_CMD_USER_2 1/2 0 to start a simulated moving target
// -- then, test module will produce mission msg continiously, include the target loc (leader location)
// -- sub step 1: use MAV_CMD_USER_2 0 1/2/3/4 to send ONE msg to follow/insert wp/takeoff ...
void Plane::test_HB1_follow(int16_t msg_id, int16_t msg_id_second, int16_t msg_id_third)
{
    if (msg_id_second == 0 && msg_id_third == 0) {HB1_test.status = msg_id;}
    if (msg_id == 0 && msg_id_third == 0) {HB1_test.cmd_type = msg_id_second;}
    if (msg_id == 0 && msg_id_second == 0) {HB1_test.cmd_type2 = msg_id_third;}
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
    if (HB1_test.status != 0) {test_HB1_mission_update_msg();}
}

void Plane::test_HB1_follow_target_update_1(float t_ms)
{
    const float x_len = 50000.0f;
    const float y_len = 100000.0f;
    float tmp_vel = 4500.0f;
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
        HB1_test.follow_dir = tmp_dir_ang;
    }

}


void Plane::test_HB1_follow_target_update_2(float t_ms)
{
    float tmp_vel = 4500.0f;
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
    HB1_test.follow_dir = tmp_dir_ang;

}

void Plane::test_HB1_follow_target_reset(void)
{
    Vector3f tmp_target(0.0f, 0.0f, 10000.f);
    Location loc(tmp_target);
    HB1_test.follow_dir = 0.0f;
    HB1_test.follow_loc.lng = loc.lng;
    HB1_test.follow_loc.lat = loc.lat;
    HB1_test.follow_loc.set_alt_cm(tmp_target.z, Location::AltFrame::ABOVE_HOME);
    HB1_test.state = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "T follow reset");
}

void Plane::test_HB1_mission_update_msg() {
    static bool in_group = false;
    static float apm_deltaX = 30.0f;
    static float apm_deltaY = 0.0f;
    static float apm_deltaZ = 0.0f;
    Location tmp_loc;
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.need_send = false;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    tmp_msg._msg_1.content.msg.in_group = in_group;
    tmp_msg._msg_1.content.msg.remote_index = 0;
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
    tmp_msg._msg_1.content.msg.control_id = 0;
    tmp_msg._msg_1.content.msg.youshang_target_airspeed = 0;
    tmp_msg._msg_1.content.msg.youshang_target_orthdist = 0;
    tmp_msg._msg_1.content.msg.youshang_target_alt = 0;
    tmp_msg._msg_1.content.msg.apm_deltaX = apm_deltaX;
    tmp_msg._msg_1.content.msg.apm_deltaY = apm_deltaY;
    tmp_msg._msg_1.content.msg.apm_deltaZ = apm_deltaZ;
    tmp_msg._msg_1.content.msg.leader_lng = (int32_t)((double)HB1_test.follow_loc.lng*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_lat = (int32_t)((double)HB1_test.follow_loc.lat*tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_alt = (int16_t)((float)HB1_test.follow_loc.alt*0.01f*tmp_msg.SF_ALT);
    tmp_msg._msg_1.content.msg.leader_dir = (int16_t)(wrap_180(HB1_test.follow_dir)*tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.leader_target_id = 0;
    tmp_msg._msg_1.content.msg.net_timeout = false;
    // tmp_msg._msg_1.content.msg.target_vx = 0;
    // tmp_msg._msg_1.content.msg.target_vy = 0;
    // tmp_msg._msg_1.content.msg.target_vz = 0;
    
    switch (HB1_test.cmd_type) {
        case 1: // cmd takeoff
            tmp_msg._msg_1.content.msg.remote_index = 0x63;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM takeoff");
            break;
        case 2: // cmd follow
            in_group = true;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM follow");
            break;
        case 3: // cmd away
            in_group = false;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM away");
            break;
        case 4: // cmd pre attack
            in_group = false;
            tmp_msg._msg_1.content.msg.remote_index = 0x69;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_preattack.time_s = 3;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM pre attack");
            break;
        case 5: // cmd attack
            in_group = false;
            tmp_msg._msg_1.content.msg.remote_index = 0xE5;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM attack");
            break;
        case 11: // insert wp
            tmp_msg._msg_1.content.msg.remote_index = 0x9C;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index = 1;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index = 0;
            tmp_loc = test_HB1_generate_wp();
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM set wp");
            break;
        case 12: // insert interim
            tmp_msg._msg_1.content.msg.remote_index = 0x66;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.p1 = 0;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.interim_point_index = 2;
            tmp_loc = test_HB1_generate_interim_attack();
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM set interim");
            break;
        case 13: // insert attack
            tmp_msg._msg_1.content.msg.remote_index = 0x33;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.p1 = 0;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.attack_point_index = 0;
            tmp_loc = test_HB1_generate_interim_attack(true);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM set attack");
            break;
        case 14: 
            apm_deltaX = 1500.0f;
            apm_deltaY = 1500.0f;
            apm_deltaZ = 1500.0f;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM Offset");
            break;

        case 21: // Speed up
            tmp_msg._msg_1.content.msg.remote_index = 0x3A;
            break;
        case 22: // Speed up
            tmp_msg._msg_1.content.msg.remote_index = 0xA7;
            break;
        case 23: // Rocket ON
            tmp_msg._msg_1.content.msg.remote_index = 0x55;
            break;
        case 24: // EngineStart
            tmp_msg._msg_1.content.msg.remote_index = 0xA5;
            break;
        case 25: // EngineOFF
            tmp_msg._msg_1.content.msg.remote_index = 0xC6;
            break;
        case 26: // EngineFULL
            tmp_msg._msg_1.content.msg.remote_index = 0xE7;
            break;
        case 27: // EngineMID
            tmp_msg._msg_1.content.msg.remote_index = 0xB4;
            break;
        case 28: // Disarm
            tmp_msg._msg_1.content.msg.remote_index = 0xCC;
            break;
        case 29: // ServoTest
            tmp_msg._msg_1.content.msg.remote_index = 0x99;
            break;
        case 30: // Search wp
            tmp_msg._msg_1.content.msg.remote_index = 0x7E;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_searchwp.line_index = 1;
            break;
        case 31: // min throttle
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 100);
            tmp_msg._msg_1.updated = false;
            tmp_msg._msg_1.need_send = false;
            tmp_msg._msg_1.print = false;
            break;
        case 32: // full throttle
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 0);
            tmp_msg._msg_1.updated = false;
            tmp_msg._msg_1.need_send = false;
            tmp_msg._msg_1.print = false;
            break;
        default:
            break;
    }

    HB1_test.cmd_type = 0;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

void Plane::test_HB1_mission_send_msg() {
    static bool in_group = false;
    static float apm_deltaX = 0.0f;
    static float apm_deltaY = 0.0f;
    static float apm_deltaZ = 0.0f;
    Location tmp_loc;
    HB1_mission2apm &tmp_msg = HB1_uart_mission.get_msg_mission2apm();
    tmp_msg._msg_1.updated = false;
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.header.head_1 = HB1_mission2apm::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_mission2apm::PREAMBLE2;
    tmp_msg._msg_1.content.msg.header.index = HB1_mission2apm::INDEX1;
    tmp_msg._msg_1.content.msg.sum_check = 0;

    tmp_msg._msg_1.content.msg.in_group = in_group;
    tmp_msg._msg_1.content.msg.remote_index = 0;
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
    tmp_msg._msg_1.content.msg.control_id = 0;
    tmp_msg._msg_1.content.msg.youshang_target_airspeed = 0;
    tmp_msg._msg_1.content.msg.youshang_target_orthdist = 0;
    tmp_msg._msg_1.content.msg.youshang_target_alt = 0;
    tmp_msg._msg_1.content.msg.apm_deltaX = apm_deltaX;
    tmp_msg._msg_1.content.msg.apm_deltaY = apm_deltaY;
    tmp_msg._msg_1.content.msg.apm_deltaZ = apm_deltaZ;
    tmp_msg._msg_1.content.msg.leader_lng = (int32_t)((double)current_loc.lng * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_lat = (int32_t)((double)current_loc.lat * tmp_msg.SF_LL);
    tmp_msg._msg_1.content.msg.leader_alt = (int16_t)(relative_ground_altitude(false) * tmp_msg.SF_ALT);

    tmp_msg._msg_1.content.msg.leader_dir = (int16_t)(gps.ground_course_cd()*0.01f * tmp_msg.SF_ANG);
    tmp_msg._msg_1.content.msg.leader_target_id = mission.get_current_nav_index();
    tmp_msg._msg_1.content.msg.net_timeout = false;
    // tmp_msg._msg_1.content.msg.target_vx = 0;
    // tmp_msg._msg_1.content.msg.target_vy = 0;
    // tmp_msg._msg_1.content.msg.target_vz = 0;
    
    switch (HB1_test.cmd_type2) {
        case 1: // cmd takeoff
            tmp_msg._msg_1.content.msg.remote_index = 0x63;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out takeoff");
            break;
        case 2: // cmd follow
            in_group = true;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out follow");
            break;
        case 3: // cmd away
            in_group = false;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out away");
            break;
        case 4: // cmd pre attack
            in_group = false;
            tmp_msg._msg_1.content.msg.remote_index = 0x69;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out pre attack attack");
            break;
        case 5: // cmd attack
            in_group = false;
            tmp_msg._msg_1.content.msg.remote_index = 0xE5;
            //tmp_msg._msg_1.content.msg.remote_index = 0xA3;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out attack");
            break;
        case 11: // insert wp
            tmp_msg._msg_1.content.msg.remote_index = 0x9C;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index = 0;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index = 0;
            tmp_loc = test_HB1_generate_wp();
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out set wp");
            break;
        case 12: // insert interim
            tmp_msg._msg_1.content.msg.remote_index = 0x66;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.p1 = 0;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.interim_point_index = 2;
            tmp_loc = test_HB1_generate_interim_attack();
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_interim.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out set interim");
            break;
        case 13: // insert attack
            tmp_msg._msg_1.content.msg.remote_index = 0x33;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.p1 = 0;
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.attack_point_index = 0;
            tmp_loc = test_HB1_generate_interim_attack(true);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.longitude = (int32_t)((double)tmp_loc.lng*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.latitude = (int32_t)((double)tmp_loc.lat*tmp_msg.SF_LL);
            tmp_msg._msg_1.content.msg.remote_cmd.cmd_attack.alt = (int16_t)((float)tmp_loc.alt*0.01f*tmp_msg.SF_ALT);
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out set attack");
            break;
        case 21: 
            apm_deltaX = (int16_t)150.0f;
            apm_deltaY = (int16_t)150.0f;
            apm_deltaZ = (int16_t)30.0f;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out Offset");
            break;
        case 22: 
            apm_deltaX = (int16_t)0.0f;
            apm_deltaY = (int16_t)0.0f;
            apm_deltaZ = (int16_t)0.0f;
            gcs().send_text(MAV_SEVERITY_INFO, "SIM out Offset cancel");
        default:
            break;
    }

    HB1_test.cmd_type2 = 0;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
}

Location Plane::test_HB1_generate_wp() {
    static int16_t index_loc = 0;
    float alt = 27000.0f;
    Vector3f tmp_xyz = Vector3f(0.0f, 0.0f, alt);
    switch (index_loc) {
        case 0 :
            tmp_xyz = Vector3f(100000.0f, 0.0f, alt);
            break;
        case 1 :
            tmp_xyz = Vector3f(0.0f, 100000.0f, alt);
            break;
        default:
            tmp_xyz = Vector3f(100000.0f * (float)index_loc, 100000.0f * (float)index_loc, alt);
            break;
    }
    index_loc++;
    Location tmp_loc(tmp_xyz);
    return tmp_loc;
}

Location Plane::test_HB1_generate_interim_attack(bool is_attack) {
    static int16_t index_loc = 0;
    float alt = 45000.0f;
    Vector3f tmp_xyz = Vector3f(0.0f, 0.0f, alt);
    switch (index_loc) {
        case 0 :
            tmp_xyz = Vector3f(-200000.0f, 0.0f, alt);
            break;
        case 1 :
            tmp_xyz = Vector3f(0.0f, -200000.0f, alt);
            break;
        default:
            tmp_xyz = Vector3f(-200000.0f * (float)index_loc, -200000.0f * (float)index_loc, alt);
            break;
    }
    if (is_attack) {tmp_xyz.z = 10000.0f;}
    index_loc++;
    Location tmp_loc(tmp_xyz);
    return tmp_loc;
}