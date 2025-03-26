#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//

FD_Target_LRB::FD_Target_LRB()
{
    ;
}

bool FD_Target_LRB::init() {
    _last_ms = 0;
    _valid = false;
    FD_LRB_ptr = new FD_LRB(AP_SerialManager::SerialProtocol_CAM);
    FD_LRB_ptr->get_msg_cam_cmd().set_enable();
    FD_LRB_ptr->get_msg_cam_status().set_enable();
    FD_LRB_ptr->get_msg_cam_target().set_enable();
    return FD_LRB_ptr->initialized();
}

void FD_Target_LRB::update() {
    static uint32_t last_update_ms = millis();

    FD_LRB_ptr->read();
    FD_LRB_TARGET &tmp_msg = FD_LRB_ptr->get_msg_cam_target();
    if (tmp_msg._msg_1.updated) {
        // DYT -> APM
        if (tmp_msg._msg_1.content.msg.status == 1) {
            float p1 =  cal_frame_angle(cam_width.get(), cam_angle_x.get(), (tmp_msg._msg_1.content.msg.target_x + tmp_msg._msg_1.content.msg.target_w/2 - cam_x_offset.get()) ); // x-axis, degree
            float p2 = -cal_frame_angle(cam_height.get(), cam_angle_y.get(), (tmp_msg._msg_1.content.msg.target_y + tmp_msg._msg_1.content.msg.target_h/2- cam_y_offset.get()) ); // y-axis, degree

            // p2 += cam_pitch_offset.get(); // add offset between cam and uav
            handle_info(p1, p2);
        } else {
            // unhealthy massage
        }
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
        if (!is_valid()) {
            do_cmd_pre_lock();
        }
    }
}

float FD_Target_LRB::cal_frame_angle(float pixel, float angle, float x_in)
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

void FD_Target_LRB::do_cmd_on(bool on) {
    if (on) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Lock ON");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Lock OFF");
    }

    FD_LRB_CMD &tmp_msg = FD_LRB_ptr->get_msg_cam_cmd();

    tmp_msg._msg_1.content.msg.header.head_1 = FD_LRB_CMD::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD_LRB_CMD::PREAMBLE2;
    tmp_msg._msg_1.content.msg.length = 0x10;
    tmp_msg._msg_1.content.msg.frametype = 0x69;
    tmp_msg._msg_1.content.msg.on = on?0x01:0x02;
    tmp_msg._msg_1.content.msg.type = 0x01;
    tmp_msg._msg_1.content.msg.size = (uint8_t)lock_size;
    int16_t lock_x_center_corr = 0;
    int16_t lock_y_center_corr = 0;
    switch (lock_size.get()) {
        case 1:
            lock_x_center_corr = 8;
            lock_y_center_corr = 8;
            break;
        case 2:
            lock_x_center_corr = 16;
            lock_y_center_corr = 16;
            break;
        case 3:
            lock_x_center_corr = 32;
            lock_y_center_corr = 32;
            break;
        case 4:
            lock_x_center_corr = 64;
            lock_y_center_corr = 64;
            break;
    }

    int16_t lock_y_offset = 0;
    switch (lock_y_down.get()) {
        case 1:
            lock_y_offset = 16;
            break;
        case 2:
            lock_y_offset = 32;
            break;
        case 3:
            lock_y_offset = 64;
            break;
        case 4:
            lock_y_offset = 128;
            break;
    }

    tmp_msg._msg_1.content.msg.target_x = (int16_t)lock_x - lock_x_center_corr;
    tmp_msg._msg_1.content.msg.target_y = (int16_t)lock_y - lock_y_center_corr + lock_y_offset;

    tmp_msg.make_sum();
    tmp_msg._msg_1.need_send = true;

    FD_LRB_ptr->write();
}

void FD_Target_LRB::do_cmd_pre_lock() {
    FD_LRB_CMD &tmp_msg = FD_LRB_ptr->get_msg_cam_cmd();

    tmp_msg._msg_1.content.msg.header.head_1 = FD_LRB_CMD::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD_LRB_CMD::PREAMBLE2;
    tmp_msg._msg_1.content.msg.length = 0x10;
    tmp_msg._msg_1.content.msg.frametype = 0x69;
    tmp_msg._msg_1.content.msg.on = 0x00;
    tmp_msg._msg_1.content.msg.type = 0x01;
    tmp_msg._msg_1.content.msg.size = (uint8_t)lock_size;
    int16_t lock_x_center_corr = 0;
    int16_t lock_y_center_corr = 0;
    switch (lock_size.get()) {
        case 1:
            lock_x_center_corr = 8;
            lock_y_center_corr = 8;
            break;
        case 2:
            lock_x_center_corr = 16;
            lock_y_center_corr = 16;
            break;
        case 3:
            lock_x_center_corr = 32;
            lock_y_center_corr = 32;
            break;
        case 4:
            lock_x_center_corr = 64;
            lock_y_center_corr = 64;
            break;
    }

    int16_t lock_y_offset = 0;
    switch (lock_y_down.get()) {
        case 1:
            lock_y_offset = 16;
            break;
        case 2:
            lock_y_offset = 32;
            break;
        case 3:
            lock_y_offset = 64;
            break;
        case 4:
            lock_y_offset = 128;
            break;
    }

    tmp_msg._msg_1.content.msg.target_x = (int16_t)lock_x - lock_x_center_corr;
    tmp_msg._msg_1.content.msg.target_y = (int16_t)lock_y - lock_y_center_corr + lock_y_offset;

    tmp_msg.make_sum();
    tmp_msg._msg_1.need_send = true;

    FD_LRB_ptr->write();
}

void FD_Target_LRB::handle_info_test(float p1, float p2) {
    handle_info(p1, p2);
    // FD_LRB_TARGET &tmp_msg = FD_LRB_ptr->get_msg_cam_target();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    // tmp_msg._msg_1.content.msg.status = 1;
}
