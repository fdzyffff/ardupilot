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
    FD1_uart_RK3588.init();
    FD1_uart_SIYIA8.init();

    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.p21 = 0.0f;
    display_info.p22 = 0.0f;
    display_info.p23 = 0.0f;

    front_status.dist_cm = 0.0f;
    front_status.bf_info.zero();
    front_status.yaw_rate = 0.0f;
    front_status.vel.zero();
    front_status.count = 0;
    front_status.approached = false;
    up_status.dist_cm = 0.0f;
    up_status.bf_info.zero();
    // up_status.efb_info.zero();
    // up_status.efb_info_filt.set_cutoff_frequency(20.f, copter.g2.user_parameters.filt_hz.get());
    up_status.yaw_rate = 0.0f;
    up_status.bf_vel.zero();
    up_status.count = 0;

    up_status.bf_wind_x_filter.init(50, 50);
    up_status.bf_wind_y_filter.init(50, 50);
    // gcs().send_text(MAV_SEVERITY_INFO, "FD1_uart_RK3588.init()");
}

void UA8::update_log() {
    AP::logger().WriteStreaming("UTU1",
                                "TimeUS,bfx,bfy,bwx,bwy,dist,yawr,bfvx,bfvy,fps",
                                "s---------",
                                "F---------",
                                "Qfffffffff",
                                AP_HAL::micros64(),
                                (float)up_status.bf_info.x,
                                (float)up_status.bf_info.y,
                                (float)up_status.bf_wind.x,
                                (float)up_status.bf_wind.y,
                                (float)up_status.dist_cm,
                                (float)up_status.yaw_rate,
                                (float)up_status.bf_vel.x,
                                (float)up_status.bf_vel.y,
                                (float)up_status.fps);

    AP::logger().WriteStreaming("UTU2",
                                "TimeUS,bwxr,bwyr",
                                "s--",
                                "F--",
                                "Qff",
                                AP_HAL::micros64(),
                                (float)up_status.bf_wind_raw.x,
                                (float)up_status.bf_wind_raw.y);

    AP::logger().WriteStreaming("UTF1",
                                "TimeUS,bfx,bfy,dist,yawr,bfvx,bfvy,bfvz,fps",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)front_status.bf_info.x,
                                (float)front_status.bf_info.y,
                                (float)front_status.dist_cm,
                                (float)front_status.yaw_rate,
                                (float)front_status.vel.x,
                                (float)front_status.vel.y,
                                (float)front_status.vel.z,
                                (float)front_status.fps);

    AP::logger().WriteStreaming("UTG1",
                                "TimeUS,roll,pitch,yaw,fps",
                                "s----",
                                "F----",
                                "Qffff",
                                AP_HAL::micros64(),
                                (float)gimbal_status.roll,
                                (float)gimbal_status.pitch,
                                (float)gimbal_status.yaw,
                                (float)gimbal_status.fps);
}

void UA8::read_uart()
{
    if (FD1_uart_RK3588.get_port() != nullptr) {
        while (FD1_uart_RK3588.get_port()->available()>0) {
            uint8_t temp = FD1_uart_RK3588.get_port()->read();
            uart_msg_RK3588.parse(temp);
            if (uart_msg_RK3588._msg_1.updated) {
                handle_RK3588();
                update_log();
            }
        }
    }
    if (FD1_uart_SIYIA8.get_port() != nullptr) {
        while (FD1_uart_SIYIA8.get_port()->available()>0) {
            uint8_t temp = FD1_uart_SIYIA8.get_port()->read();
            uart_msg_SIYIA8mini.parse(temp);
            if (uart_msg_SIYIA8mini._msg_1.updated) {
                handle_SIYIA8mini();
            }
        }
    }
}

void UA8::handle_RK3588()
{
    if (uart_msg_RK3588._msg_1.content.msg.tag_ok) {
        float p1 = cal_frame_angle(copter.g2.user_parameters.cam_angle_x.get(), uart_msg_RK3588._msg_1.content.msg.norm_x); // x-axis, degree
        float p2 = cal_frame_angle(copter.g2.user_parameters.cam_angle_y.get(), uart_msg_RK3588._msg_1.content.msg.norm_y); // y-axis, degree
        float p3 = uart_msg_RK3588._msg_1.content.msg.att_yaw;
        Vector3f dist_vec = Vector3f(uart_msg_RK3588._msg_1.content.msg.dist_x, uart_msg_RK3588._msg_1.content.msg.dist_y, uart_msg_RK3588._msg_1.content.msg.dist_z);
        uint32_t tag_id = uart_msg_RK3588._msg_1.content.msg.tag_id;
        float dist = dist_vec.length() * sclae_factor_by_id(tag_id);

        display_info.p1 = p1;
        display_info.p2 = p2;
        display_info.p3 = p3;
        display_info.p4 = dist;

        uint8_t type = 0;
        if (tag_id < 100) {
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

float UA8::sclae_factor_by_id(uint32_t id)
{
    float ret = 1.0f;
    if (id<=50) {
        ret = copter.g2.user_parameters.tag_scale_f_big.get();
    }
    if (51<=id && id<=100) {
        ret = copter.g2.user_parameters.tag_scale_f_big.get() * copter.g2.user_parameters.tag_scale_f_small.get();
    }
    if (101<=id && id<=150) {
        ret = copter.g2.user_parameters.tag_scale_u_big.get();
    }
    if (151<=id && id<=200) {
        ret = copter.g2.user_parameters.tag_scale_u_big.get() * copter.g2.user_parameters.tag_scale_u_small.get();
    }
    return ret;
}

void UA8::handle_SIYIA8mini()
{
    if (uart_msg_SIYIA8mini._msg_1.content.msg.cmd_id == 0x0D) {
        gimbal_status.last_ms = millis();
        gimbal_status.yaw = (float)uart_msg_SIYIA8mini._msg_1.content.msg_0x0D.yaw * 0.1f;
        gimbal_status.pitch = (float)uart_msg_SIYIA8mini._msg_1.content.msg_0x0D.pitch * 0.1f;
        gimbal_status.roll = (float)uart_msg_SIYIA8mini._msg_1.content.msg_0x0D.roll * 0.1f;
        gimbal_status.count++;
    }
    uart_msg_SIYIA8mini._msg_1.updated = false; 
}

float UA8::cal_frame_angle(float angle, float x_in)
{
    // pixel, eg: 1080
    // angle, eg: 54°
    // x_in, eg: 540
    // ret, eg: 0°
    angle = constrain_float(radians(angle), radians(10.0f), radians(150.0f));
    x_in = constrain_float(x_in, -1.0f, 1.0f);
    float ret = atanf(x_in*tanf(angle*0.5f));
    return degrees(ret);
}

void UA8::handle_front_info(float p1, float p2, float p3, float dist) {
    front_status.valid = true;
    front_status.last_ms = millis();
    front_status.count++;

    front_status.bf_info.x = p1 + gimbal_status.yaw; // yaw degree
    front_status.bf_info.y = p2; // pitch degree
    front_status.dist_cm = dist * 100.f;

    update_front_vel();
    update_front_yaw_rate();
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

    float gimbal_yaw = 90.f;
    Vector3f tmp_gimbal_info = Vector3f(p1, p2, wrap_180(-p3 + gimbal_status.yaw + gimbal_yaw));
    Matrix3f tmp_gimbal_m;
    tmp_gimbal_m.from_euler(0.0f, 0.0f, radians(90.0f));// gimbal 90 rotate in yaw
    up_status.bf_info = tmp_gimbal_m * tmp_gimbal_info;

    up_status.bf_info.z = wrap_180(-p3 + gimbal_status.yaw + gimbal_yaw);

    display_info.p11 = up_status.bf_info.x;
    display_info.p12 = up_status.bf_info.y;
    display_info.p13 = up_status.bf_info.z;

    up_status.dist_cm = dist * 100.f;

    update_up_yaw_rate();

    update_wind_comp();
    update_up_bf_vel_x_ms();
    update_up_bf_vel_y_ms();
    // display_info.p31 = get_target_vel_x_ms();
    // display_info.p32 = get_target_vel_y_ms();
}

// update 
void UA8::update()
{
    read_uart();
    update_valid();
    update_init();

}

void UA8::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - front_status.last_ms) > _time_out)||(front_status.last_ms == 0) ) )  {
        if (front_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Front lost");
        }
        front_status.dist_cm = 0.0f;
        front_status.bf_info.zero();
        front_status.yaw_rate = 0.0f;
        front_status.vel.zero();
        front_status.count = 0;
        front_status.approached = false;

        front_status.valid = false;
    } else {
        if (!front_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Front aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        front_status.valid = true;
    }

    if (_time_out != 0 && ( ((now - up_status.last_ms) > _time_out)||(up_status.last_ms == 0) ) )  {
        if (up_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Up lost");
        }
        up_status.dist_cm = 0.0f;
        up_status.bf_info.zero();
        up_status.bf_wind.zero();
        up_status.yaw_rate = 0.0f;
        up_status.bf_vel.zero();
        up_status.count = 0;

        up_status.valid = false;
    } else {
        if (!up_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Up aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        up_status.valid = true;
    }

    if (_time_out != 0 && ( ((now - gimbal_status.last_ms) > _time_out)||(gimbal_status.last_ms == 0) ) )  {
        if (gimbal_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal lost");
        }
        gimbal_status.zoom = 0;
        gimbal_status.roll = 0;
        gimbal_status.pitch = 0;
        gimbal_status.yaw = 0;
        gimbal_status.count = 0;

        gimbal_status.valid = false;
    } else {
        if (!gimbal_status.valid) {
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        gimbal_status.valid = true;
    }
    if (gimbal_status.valid == false) {
        if (now - gimbal_status.last_send_ms > 1000) {
            gimbal_status.last_send_ms = now;
            set_attitude_hz();
        }
    }
}

void UA8::update_init()
{
    uint32_t now = millis();
    if ((gimbal_status.last_init_count < 3) && (now - gimbal_status.last_init_ms > 1000)) {
        gimbal_status.last_init_ms = now;
        gimbal_status.last_init_count++;
        set_attitude_hz();
        set_gimbal_front();
    }
}

// m/s
void UA8::update_front_vel()
{
    float k = copter.g2.user_parameters.attack_k.get();
    Vector3f tmp_vel = Vector3f((front_status.dist_cm - copter.g2.user_parameters.approach_cm.get()) * 0.01f * k, 0.0f, 0.0f);
    Matrix3f tmp_gimbal_m;
    tmp_gimbal_m.from_euler(0.0f, 0.0f, radians(90.0f));// gimbal 90 rotate in yaw
    Vector3f bf_vel = tmp_gimbal_m * tmp_vel;

    front_status.vel.x = constrain_float(bf_vel.x, -0.3f, 0.3f);
    front_status.vel.y = constrain_float(bf_vel.y, -0.3f, 0.3f);

    float angle_comp = constrain_float(front_status.bf_info.y, -30.0f, 30.0f);
    front_status.vel.z = constrain_float(sinf(radians(angle_comp)) * front_status.dist_cm * 0.01f, -0.3f, 0.3f);

    bool ret1 = fabsf(front_status.dist_cm - copter.g2.user_parameters.approach_cm.get()) < 30.f;
    bool ret2 = front_status.valid;
    front_status.approached = ret1&&ret2;
    // gcs().send_text(MAV_SEVERITY_INFO, "(%f)front_vel_xy().length() %f|%f", k, copter.ua8.get_front_vel_xy().length(), front_status.vel.xy().length());
}

// degree/second
void UA8::update_front_yaw_rate()
{
    float k2 = copter.g2.user_parameters.attack_k2.get();
    float angle_comp = constrain_float(wrap_180(front_status.bf_info.x + gimbal_status.yaw), -15.0f, 15.0f);
    front_status.yaw_rate = k2 * angle_comp; // degrees/s
}

// degree/second
void UA8::update_up_yaw_rate()
{
    float k2 = copter.g2.user_parameters.attack_k2.get();
    float angle_comp = constrain_float(up_status.bf_info.z, -15.0f, 15.0f);
    up_status.yaw_rate = k2 * angle_comp; // degrees/s
}

// m/s
void UA8::update_up_bf_vel_x_ms()
{
    float k = copter.g2.user_parameters.attack_k.get();
    float dist_r = constrain_float(up_status.dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(-(up_status.bf_info.y - up_status.bf_wind.y + copter.g2.user_parameters.cam_pitch_off), -15.0f, 15.0f)));
    // float dist = dist_r*tanf(radians(constrain_float(-bf_info.y, -15.0f, 15.0f)));
    up_status.bf_vel.x = k * dist; // degrees/s
}

// m/s
void UA8::update_up_bf_vel_y_ms()
{
    float k = copter.g2.user_parameters.attack_k.get();
    float dist_r = constrain_float(up_status.dist_cm*0.01f, 0.0f, 1.0f);
    float dist = dist_r*tanf(radians(constrain_float(copter.g2.user_parameters.cam_roll_off + up_status.bf_info.x - up_status.bf_wind.x, -15.0f, 15.0f)));
    // float dist = dist_r*tanf(radians(constrain_float(bf_info.x, -15.0f, 15.0f)));
    up_status.bf_vel.y = k * dist; // degrees/s
}

void UA8::update_wind_comp()
{
    Matrix3f tmp_ef_m;
    Vector3f tmp_input = Vector3f(copter.pos_control->get_vel_xy_pid().get_pid_info_x().I, copter.pos_control->get_vel_xy_pid().get_pid_info_y().I, 0.0f);
    tmp_ef_m.from_euler(0.0f, 0.0f, copter.ahrs_view->yaw);
    tmp_ef_m.transpose();
    Vector3f tmp_output = tmp_ef_m*tmp_input;

   //x of wind is in y direction of body cam
    up_status.bf_wind_raw.y = -degrees(atanf(tmp_output.x*0.001f));
    up_status.bf_wind_raw.x =  degrees(atanf(tmp_output.y*0.001f));

    up_status.bf_wind_x_filter.push(up_status.bf_wind_raw.x);
    up_status.bf_wind_y_filter.push(up_status.bf_wind_raw.y);

    up_status.bf_wind.x = up_status.bf_wind_x_filter.get() * 0.5f;
    up_status.bf_wind.y = up_status.bf_wind_y_filter.get() * 0.5f;
}

void UA8::set_gimbal_front()
{
    if (FD1_uart_SIYIA8.get_port() == nullptr) {
        return;
    }
    uart_msg_SIYIA8mini.pack_stabilize_mode();
    FD1_uart_SIYIA8.get_port()->write(uart_msg_SIYIA8mini._msg_1.content.data, uart_msg_SIYIA8mini._msg_1.content.msg.data_length+10);
    uart_msg_SIYIA8mini.pack_center();
    FD1_uart_SIYIA8.get_port()->write(uart_msg_SIYIA8mini._msg_1.content.data, uart_msg_SIYIA8mini._msg_1.content.msg.data_length+10);
}

void UA8::set_gimbal_up()
{
    if (FD1_uart_SIYIA8.get_port() == nullptr) {
        return;
    }
    uart_msg_SIYIA8mini.pack_stabilize_mode();
    FD1_uart_SIYIA8.get_port()->write(uart_msg_SIYIA8mini._msg_1.content.data, uart_msg_SIYIA8mini._msg_1.content.msg.data_length+10);
    uart_msg_SIYIA8mini.pack_angle(0.0f, -90.f);
    FD1_uart_SIYIA8.get_port()->write(uart_msg_SIYIA8mini._msg_1.content.data, uart_msg_SIYIA8mini._msg_1.content.msg.data_length+10);
}

void UA8::set_attitude_hz()
{
    if (FD1_uart_SIYIA8.get_port() == nullptr) {
        return;
    }
    uart_msg_SIYIA8mini.pack_attitude_hz();
    FD1_uart_SIYIA8.get_port()->write(uart_msg_SIYIA8mini._msg_1.content.data, uart_msg_SIYIA8mini._msg_1.content.msg.data_length+10);
}

bool UA8::have_target_front()
{
    return front_status.valid;
}

bool UA8::have_target_up()
{
    return up_status.valid;
}

uint8_t UA8::is_valid()
{
    uint8_t ret = 0;
    if (front_status.valid) {
        ret = 1;
    }
    if (up_status.valid) {
        ret = 1;
    }
    return ret;
}

void UA8::test()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "vel_x_I %f", copter.pos_control->get_vel_xy_pid().get_pid_info_x().I);
    // gcs().send_text(MAV_SEVERITY_INFO, "vel_y_I %f", copter.pos_control->get_vel_xy_pid().get_pid_info_y().I);

    if (copter.g2.user_parameters.test_cam.get() == 0) {
        return;
    }
    static uint32_t test_count = 0;
    if ((test_count%10)==0 && (test_count/10)%2 == 0) {
        set_gimbal_front();
    }

    if ((test_count%10)==0 && (test_count/10)%2 == 1) {
        set_gimbal_up();
    }

    test_count++;
}

void UA8::do_print()
{
    if ((copter.g2.user_parameters.cam_print.get() & (1<<0)) && (front_status.count + up_status.count)) { // 1
        gcs().send_text(MAV_SEVERITY_INFO, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", (front_status.count + up_status.count), display_info.p1, display_info.p2, display_info.p3, display_info.p4);
    }
    if (copter.g2.user_parameters.cam_print.get() & (1<<1)) { // 2
        if (have_target_front()) {
            gcs().send_text(MAV_SEVERITY_INFO, "f_bf [%0.1f](%0.0f,%0.0f,%0.0f)", front_status.dist_cm, front_status.bf_info.x, front_status.bf_info.y, front_status.bf_info.z);
        }
        if (have_target_up()) {
            gcs().send_text(MAV_SEVERITY_INFO, "u_bf [%0.1f](%0.0f,%0.0f,%0.0f)", up_status.dist_cm, up_status.bf_info.x, up_status.bf_info.y, up_status.bf_info.z);
        }
    }
    if (copter.g2.user_parameters.cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_INFO, "cam [%d](%0.1f,%0.1f,%0.1f)", gimbal_status.count, gimbal_status.roll, gimbal_status.pitch, gimbal_status.yaw);
    }
    // if (copter.g2.user_parameters.cam_print.get() & (1<<3)) { // 8
    //     gcs().send_text(MAV_SEVERITY_INFO, "xyd (%0.1f,%0.1f,%0.1f)", get_target_bf_vel_x(), get_target_bf_vel_y(), get_target_dist_cm());
    // }
    // if (copter.g2.user_parameters.cam_print.get() & (1<<4)) { // 16
    //     gcs().send_text(MAV_SEVERITY_INFO, "G_rpy (%0.1f,%0.1f,%0.1f)", gimbal_status.roll, gimbal_status.pitch, gimbal_status.yaw);
    // }

    up_status.fps = up_status.count;
    front_status.fps = front_status.count;
    gimbal_status.fps = gimbal_status.count;
    front_status.count = 0;
    up_status.count = 0;
    gimbal_status.count = 0;
}