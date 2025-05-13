/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Plane.h"

UK230::UK230()
{
    ;
}

// initialise
void UK230::init()
{
    _last_ms = 0;
    _valid = false;
    // _filter_target_cm.set_cutoff_frequency(30.0f, 20.f);
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p21 = 0.0f;
    display_info.p22 = 0.0f;
    display_info.p23 = 0.0f;
    display_info.count = 0;
    display_info.new_data = false;
    _target_yaw_rate = 0.0f;
    FD1_uart_K230.init();
    FD1_uart_K230.get_msg_K230().set_enable();
    // gcs().send_text(MAV_SEVERITY_INFO, "FD1_uart_K230.init()");
}

void UK230::read_uart()
{
    FD1_uart_K230.read();
    FD1_msg_K230 &tmp_msg = FD1_uart_K230.get_msg_K230();
    if (tmp_msg._msg_1.updated) {
        display_info.new_data = true;

        if (tmp_msg._msg_1.content.msg.tag_ok) {
            _last_ms = millis();
            float p1 = cal_frame_angle(plane.g2.user_cam_width.get(), plane.g2.user_cam_angle_x.get(), tmp_msg._msg_1.content.msg.tag_x); // x-axis, degree
            float p2 = cal_frame_angle(plane.g2.user_cam_height.get(), plane.g2.user_cam_angle_y.get(), tmp_msg._msg_1.content.msg.tag_y); // y-axis, degree
            float p3 = -tmp_msg._msg_1.content.msg.tag_heading;
            _target_dist_cm = tmp_msg._msg_1.content.msg.tag_d;

            display_info.p1 = tmp_msg._msg_1.content.msg.tag_x;
            display_info.p2 = tmp_msg._msg_1.content.msg.tag_y;
            display_info.p3 = tmp_msg._msg_1.content.msg.tag_heading;
            display_info.p4 = tmp_msg._msg_1.content.msg.tag_d;
            handle_info(p1, p2, p3);
        }

        tmp_msg._msg_1.updated = false;   
    }
}

float UK230::cal_frame_angle(float pixel, float angle, float x_in)
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

void UK230::handle_info(float p1, float p2, float p3) {
        // if (!_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "IIvalid %ld|%ld", millis(), _last_ms);
        // }
    _valid = true;
    _last_ms = millis();
    display_info.count++;

    Vector3f tmp_cam_input = Vector3f(radians(p1), radians(p2), radians(p3));
    Matrix3f tmp_cam_m;
    tmp_cam_m.from_euler(0.0f, 0.0f, radians(180.0f));
    Vector3f tmp_cam_output = tmp_cam_m*tmp_cam_input;

    bf_info.x = degrees(tmp_cam_output.x); // roll degree
    bf_info.y = degrees(tmp_cam_output.y); // pitch degree
    bf_info.z = wrap_180(degrees(tmp_cam_output.z) + 0.0f);; // yaw degree


    display_info.p11 = bf_info.x;
    display_info.p12 = bf_info.y;
    display_info.p13 = bf_info.z;

    update_target_yaw_rate();
    display_info.p23 = get_target_yaw_rate();

    Matrix3f tmp_bf_m;
    tmp_bf_m.from_euler(radians(bf_info.x), radians(bf_info.y), 0.0f);
    Matrix3f tmp_body_m;
    tmp_body_m.from_euler(plane.ahrs.get_roll(), plane.ahrs.get_pitch(), 0.0f);
    Matrix3f tmp_efbf_m = tmp_body_m*tmp_bf_m;
    tmp_efbf_m.to_euler(&efb_info.x, &efb_info.y, &efb_info.z);
    efb_info.x = degrees(efb_info.x);
    efb_info.y = degrees(efb_info.y);
    efb_info.z = bf_info.z;
    // efb_info = tmp_body_m*bf_info;
    update_target_bf_vel_x_ms();
    update_target_bf_vel_y_ms();
    update_target_ef_vel_ms();
    // display_info.p31 = get_target_vel_x_ms();
    // display_info.p32 = get_target_vel_y_ms();
}

// update 
void UK230::update()
{
    read_uart();
    update_valid();
    update_control();
}

void UK230::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)plane.g2.user_cam_time_out.get();
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target lost");
        }
        _valid = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        // _raw_target_cm.zero();
        // _filter_target_cm.reset();

        _target_yaw_rate = 0.0f;
        _target_bf_vel_x = 0.0f;
        _target_bf_vel_y = 0.0f;
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }
}

// degree/second
void UK230::update_target_yaw_rate() {
    float k2 = plane.g2.user_attack_k2.get();
    float angle_comp = constrain_float(bf_info.z, -15.0f, 15.0f);
    _target_yaw_rate = k2 * angle_comp; // degrees/s
}

// m/s
void UK230::update_target_bf_vel_x_ms() {
    float k = plane.g2.user_attack_k.get();
    float dist_r = constrain_float(_target_dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(-efb_info.y, -15.0f, 15.0f)));
    _target_bf_vel_x = k * dist; // degrees/s
}

// m/s
void UK230::update_target_bf_vel_y_ms() {
    float k = plane.g2.user_attack_k.get();
    float dist_r = constrain_float(_target_dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(efb_info.x, -15.0f, 15.0f)));
    _target_bf_vel_y = k * dist; // degrees/s
}

void UK230::update_target_ef_vel_ms() {
    Matrix3f tmp_body_m;
    Vector3f tmp_vel_input = Vector3f(_target_bf_vel_x, _target_bf_vel_y, 0.0f);
    tmp_body_m.from_euler(0.0f, 0.0f, plane.ahrs.get_yaw());
    Vector3f tmp_vel_output = tmp_body_m*tmp_vel_input; 
    _target_bf_vel_x = tmp_vel_output.x;
    _target_bf_vel_y = tmp_vel_output.y;
}

// update_control
void UK230::update_control()
{
    Vector2f target_velocity;

    if (!plane.position_ok()) {
        return;
    }

    if (plane.uk230.is_valid()) {
        target_velocity.x = get_target_ef_vel_x();
        target_velocity.y = get_target_ef_vel_y();
    } else {
        return;
    }
    if (plane.control_mode->mode_number() == Mode::Number::QRTL) {
        plane.set_velocity_match(target_velocity);
    }
    else if (plane.control_mode->mode_number() == Mode::Number::AUTO) {
        uint16_t id = plane.mission.get_current_nav_id();
        if (id == MAV_CMD_NAV_LAND || id == MAV_CMD_NAV_VTOL_LAND) {
            plane.set_velocity_match(target_velocity);
        }
    }
    else if (plane.control_mode->mode_number() == Mode::Number::QLOITER) {
        plane.set_velocity_match(target_velocity);
    }
}