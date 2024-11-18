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


#include "Copter.h"

UK230::UK230()
{
    FD1_uart_K230.init();
    FD1_uart_K230.get_msg_K230().set_enable();
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
    _target_pitch_rate = 0.0f;
    _target_roll_rate = 0.0f;
    _target_yaw_rate = 0.0f;
}

void UK230::read_uart()
{
    FD1_uart_K230.read();
    FD1_msg_K230 &tmp_msg = FD1_uart_K230.get_msg_K230();
    if (tmp_msg._msg_1.updated) {
        display_info.new_data = true;

        if (tmp_msg._msg_1.content.msg.tag_ok) {
            _last_ms = millis();
            float p1 =  cal_frame_angle(copter.g2.user_parameters.cam_width.get(), copter.g2.user_parameters.cam_angle_x.get(), tmp_msg._msg_1.content.msg.tag_x); // x-axis, degree
            float p2 = -cal_frame_angle(copter.g2.user_parameters.cam_height.get(), copter.g2.user_parameters.cam_angle_y.get(), tmp_msg._msg_1.content.msg.tag_y); // y-axis, degree
            float p3 = tmp_msg._msg_1.content.msg.tag_heading;
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

    bf_info.x = p1; // roll degree
    bf_info.y = p2; // pitch degree
    bf_info.z = p3; // yaw degree
    display_info.p11 = bf_info.x;
    display_info.p12 = bf_info.y;
    display_info.p13 = bf_info.z;
    display_info.count++;
    update_target_pitch_rate();
    update_target_roll_rate();
    update_target_yaw_rate();
    display_info.p21 = get_target_pitch_rate();
    display_info.p22 = get_target_roll_rate();
    display_info.p23 = get_target_yaw_rate();
}

// update 
void UK230::update()
{
    read_uart();
    update_valid();
}

void UK230::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target lost");
        }
        _valid = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        // _raw_target_cm.zero();
        // _filter_target_cm.reset();

        _target_pitch_rate = 0.0f;
        _target_roll_rate = 0.0f;
        _target_yaw_rate = 0.0f;
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }
}

// degree/second
void UK230::update_target_pitch_rate() {
    float k = copter.g2.user_parameters.attack_k.get();
    float angle_comp = constrain_float(bf_info.y, -15.0f, 15.0f);
    _target_pitch_rate = k * angle_comp; // degrees/s

    //Limit pitch rate
    float limit_pitch_rate = copter.g2.user_parameters.rate_limit.get();
    _target_pitch_rate = constrain_float(_target_pitch_rate, -limit_pitch_rate, limit_pitch_rate);

    //Limit pitch
    float current_pitch = degrees(copter.ahrs_view->pitch);
    float limit_pitch = MAX(copter.g2.user_parameters.angle_limit.get(), 0.f);
    if (current_pitch > limit_pitch) {
        _target_pitch_rate = MAX(_target_pitch_rate, 0.0f);
    } else if (current_pitch < -limit_pitch) {
        _target_pitch_rate = MIN(_target_pitch_rate, 0.0f);
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

// degree/second
void UK230::update_target_roll_rate() {
    float k = copter.g2.user_parameters.attack_k.get();
    float angle_comp = constrain_float(bf_info.y, -15.0f, 15.0f);
    _target_roll_rate = k * angle_comp; // degrees/s

    //Limit roll rate
    float limit_roll_rate = copter.g2.user_parameters.rate_limit.get();
    _target_roll_rate = constrain_float(_target_roll_rate, -limit_roll_rate, limit_roll_rate);

    //Limit roll
    float current_roll = degrees(copter.ahrs_view->roll);
    float limit_roll = MAX(copter.g2.user_parameters.angle_limit.get(), 0.f);
    if (current_roll > limit_roll) {
        _target_roll_rate = MAX(_target_roll_rate, 0.0f);
    } else if (current_roll < -limit_roll) {
        _target_roll_rate = MIN(_target_roll_rate, 0.0f);
    }
}

// degree/second
void UK230::update_target_yaw_rate() {
    float k2 = copter.g2.user_parameters.attack_k2.get();
    float angle_comp = constrain_float(bf_info.z, -15.0f, 15.0f);
    _target_yaw_rate = k2 * angle_comp; // degrees/s
}
