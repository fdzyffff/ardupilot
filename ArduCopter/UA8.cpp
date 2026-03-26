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

UA8::UA8()
{
    ;
}

// initialise
void UA8::init()
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
    FD1_uart_RK3588.init();
    uart_msg_SIYIA8mini.init();
    efb_info_filt.set_cutoff_frequency(20.f, copter.g2.user_parameters.filt_hz.get());
    // gcs().send_text(MAV_SEVERITY_INFO, "FD1_uart_RK3588.init()");
}

void UA8::read_uart()
{
    while (FD1_uart_RK3588.get_port()->available()>0) {
        uint8_t temp = FD1_uart_RK3588.get_port()->read();
        uart_msg_RK3588.parse(temp);
        if (uart_msg_RK3588._msg_1.updated) {
            handle_RK3588();
        }
        if (uart_msg_SIYIA8mini._msg_1.updated) {
            handle_SIYIA8mini();
        }
}

void UA8::handle_RK3588()
{
    display_info.new_data = true;

    if (uart_msg_RK3588._msg_1.content.msg.tag_ok) {
        _last_ms = millis();
        float p1 =  cal_frame_angle(copter.g2.user_parameters.cam_width.get(), copter.g2.user_parameters.cam_angle_x.get(), uart_msg_RK3588._msg_1.content.msg.tag_x); // x-axis, degree
        float p2 = -cal_frame_angle(copter.g2.user_parameters.cam_height.get(), copter.g2.user_parameters.cam_angle_y.get(), uart_msg_RK3588._msg_1.content.msg.tag_y); // y-axis, degree
        float p3 = uart_msg_RK3588._msg_1.content.msg.tag_heading;
        float dist = uart_msg_RK3588._msg_1.content.msg.tag_d;

        display_info.p1 = p1;
        display_info.p2 = p2;
        display_info.p3 = p3;
        display_info.p4 = dist;

        uint8_t type = 0;
        if (uart_msg_RK3588._msg_1.content.msg.tag_id < 1000) {
            type = 1;
        }
        if (type == 0) {
            handle_front_info(p1, p2, p3, dist);
        }
        if (type == 1) {
            handle_up_info(p1, p2, p3, dist);
        }
    }

    uart_msg_RK3588._msg_1.updated = false;   
}

void UA8::handle_SIYIA8mini()
{
    uart_msg_SIYIA8mini._msg_1.updated = false; 
}

float UA8::cal_frame_angle(float pixel, float angle, float x_in)
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

void UA8::handle_front_info(float p1, float p2, float p3, float dist) {
    front_status.valid = true;
    front_status.last_ms = millis();
    front_status.count++;

    front_status.bf_info.x = p1; // yaw degree
    front_status.bf_info.y = p2; // pitch degree
    front_status.dist_cm = dist;

    update_front_vel();
    update_up_yaw_rate();
    display_info.p11 = front_status.bf_info.x;
    display_info.p12 = front_status.bf_info.y;
    display_info.p13 = get_front_vel_xy().length();
    display_info.p14 = get_front_yaw_rate();
}

void UA8::handle_up_info(float p1, float p2, float p3, float dist) {
        // if (!_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "IIvalid %ld|%ld", millis(), _last_ms);
        // }
    up_status.valid = true;
    up_status.last_ms = millis();
    up_status.count++;

    up_status.bf_info.x = p1; // yaw degree
    up_status.bf_info.y = p2; // pitch degree

    up_status.dist_cm = dist;

    if (p2 < -90.f) {
        p2 = -180.0f - p2;
    } else if (p2 > 90.0f) {
        p2 = 180.0f - p2;
    }

    Matrix3f tmp_extra1_m;
    tmp_extra1_m.from_euler(0.0f, radians(-90.0f), 0.0f);
    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_body_m;

    Vector3f tmp_cam = Vector3f(radians(p1), radians(p2), 0.0f);
    tmp_cam_body_m.from_euler(0.0f, 0.0f, radians(-90.0f));
    up_status.bf_info = tmp_cam_body_m*tmp_cam;

    up_status.bf_info.x = wrap_180(degrees(up_status.bf_info.x)+copter.g2.user_parameters.cam_roll_off.get());
    up_status.bf_info.y = wrap_180(degrees(up_status.bf_info.y)+copter.g2.user_parameters.cam_pitch_off.get());
    up_status.bf_info.z = wrap_180(p3-90.f);

    display_info.p11 = up_status.bf_info.x;
    display_info.p12 = up_status.bf_info.y;
    display_info.p13 = up_status.bf_info.z;

    update_up_yaw_rate();

    display_info.p14 = get_up_yaw_rate();

    Matrix3f tmp_target_body_m;
    tmp_target_body_m.from_euler(radians(up_status.bf_info.x), radians(up_status.bf_info.y), 0.0f);
    Matrix3f tmp_earthb_m;
    tmp_earthb_m.from_euler(copter.ahrs_view->roll, copter.ahrs_view->pitch, 0.0f);
    Matrix3f tmp_efbf_m = tmp_earthb_m*tmp_target_body_m;

    Vector3f tmp_efb;
    tmp_efbf_m.to_euler(&tmp_efb.x, &tmp_efb.y, &tmp_efb.z);
    tmp_efb.x = degrees(tmp_efb.x);
    tmp_efb.y = degrees(tmp_efb.y);
    tmp_efb.z = up_status.bf_info.z;
    up_status.efb_up_info_filt.apply(tmp_efb);
    up_status.efb_up_info = up_status.efb_up_info_filt.get();
    // efb_info = tmp_body_m*up_status.bf_info;
    update_target_bf_vel_x_ms();
    update_target_bf_vel_y_ms();
    // display_info.p31 = get_target_vel_x_ms();
    // display_info.p32 = get_target_vel_y_ms();
}

// update 
void UA8::update()
{
    read_uart();
    update_valid();
}

void UA8::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - front_status.last_ms) > _time_out)||(front_status.last_ms == 0) ) )  {
        if (front_status.valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Front lost");
        }
        front_status.dist_cm = 0.0f;
        front_status.bf_info.zero();
        front_status.yaw_rate = 0.0f;
        front_status.vel.zero();
        front_status.count = 0;
    } else {
        if (!front_status.valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Front aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        front_status.valid = true;
    }

    if (_time_out != 0 && ( ((now - up_status.last_ms) > _time_out)||(up_status.last_ms == 0) ) )  {
        if (up_status.valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Up lost");
        }
        up_status.dist_cm = 0.0f;
        up_status.bf_info.zero();
        up_status.efb_info.zero();
        up_status.efb_info_filt.reset();
        up_status.yaw_rate = 0.0f;
        up_status.bf_vel.zero();
        up_status.count = 0;
    } else {
        if (!up_status.valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Up aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        up_status.valid = true;
    }
}

// m/s
void UA8::update_front_vel()
{
    front_status.vel_x = front_status.dist_cm * 1.0f;
}

// degree/second
void UA8::update_up_yaw_rate()
{
    float k2 = copter.g2.user_parameters.attack_k2.get();
    float angle_comp = constrain_float(front_status.bf_info.x, -15.0f, 15.0f);
    front_status.yaw_rate = k2 * angle_comp; // degrees/s
}

// degree/second
void UA8::update_up_yaw_rate()
{
    float k2 = copter.g2.user_parameters.attack_k2.get();
    float angle_comp = constrain_float(bf_info.z, -15.0f, 15.0f);
    up_status.yaw_rate = k2 * angle_comp; // degrees/s
}

// m/s
void UA8::update_up_bf_vel_x_ms()
{
    float k = copter.g2.user_parameters.attack_k.get();
    float dist_r = constrain_float(up_status.dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(-up_status.efb_up_info.y, -15.0f, 15.0f)));
    // float dist = dist_r*tanf(radians(constrain_float(-bf_info.y, -15.0f, 15.0f)));
    up_status.bf_vel.x = k * dist; // degrees/s
}

// m/s
void UA8::update_up_bf_vel_y_ms()
{
    float k = copter.g2.user_parameters.attack_k.get();
    float dist_r = constrain_float(up_status.dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(up_status.efb_up_info.x, -15.0f, 15.0f)));
    // float dist = dist_r*tanf(radians(constrain_float(bf_info.x, -15.0f, 15.0f)));
    up_status.bf_vel.y = k * dist; // degrees/s
}

void UA8::set_gimbal_front()
{
    if (uart_msg_RK3588.get_port() == nullptr) {
        return;
    }
    uart_msg_RK3588.get_port()->write(0xFF);
}

void UA8::set_gimbal_up()
{
    if (uart_msg_RK3588.get_port() == nullptr) {
        return;
    }
    uart_msg_RK3588.get_port()->write(0xFF);
}

bool UA8::have_target_front()
{
    return front_status.valid;
}

bool UA8::have_target_up()
{
    return up_status.valid;
}


void UA8::do_print()
{
    // if ((copter.g2.user_parameters.cam_print.get() & (1<<0)) && uk230.display_info.new_data) { // 1
    //     gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uk230.display_info.count, uk230.display_info.p1, uk230.display_info.p2, uk230.display_info.p3, uk230.display_info.p4);
    //     uk230.display_info.new_data = false;
    //     uk230.display_info.count = 0;
    // }
    // if (g2.user_parameters.cam_print.get() & (1<<1)) { // 2
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f,%0.0f) on:%d", uk230.display_info.p11, uk230.display_info.p12, uk230.display_info.p13, uk230.is_valid());
    // }
    // if (g2.user_parameters.cam_print.get() & (1<<2)) { // 4
    //     gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uk230.get_target_roll_rate(), uk230.get_target_pitch_rate(), uk230.get_target_yaw_rate());
    // }
    // if (g2.user_parameters.cam_print.get() & (1<<3)) { // 8
    //     gcs().send_text(MAV_SEVERITY_WARNING, "xyd (%0.1f,%0.1f,%0.1f)", uk230.get_target_bf_vel_x(), uk230.get_target_bf_vel_y(), uk230.get_target_dist_cm());
    // }
}