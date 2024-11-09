#include "Copter.h"

// Convenience macros //////////////////////////////////////////////////////////
//

UCam::UCam(UAttack &frotend_in, AP_HAL::UARTDriver* port_in):
    UCam_base(frotend_in)
{
    FD_CAM_ptr = new FD_CAM(port_in);
    FD_CAM_ptr->get_msg_cam_cmd().set_enable();
    FD_CAM_ptr->get_msg_cam_status().set_enable();
    FD_CAM_ptr->get_msg_cam_target().set_enable();
    _yaw_rate_filter.set_cutoff_frequency(10.f, 25.f);
    return;
}

void UCam::update() {
    static uint32_t last_update_ms = millis();
    _yaw_rate_filter.apply(degrees(AP::ahrs().get_yaw_rate_earth()));

    FD_CAM_ptr->read();
    FD_CAM_TARGET &tmp_msg = FD_CAM_ptr->get_msg_cam_target();
    if (tmp_msg._msg_1.updated) {
        // DYT -> APM
        if (tmp_msg._msg_1.content.msg.status == 1) {
            float p1 =  cal_frame_angle(copter.g2.user_parameters.cam_width.get(), copter.g2.user_parameters.cam_angle_x.get(), tmp_msg._msg_1.content.msg.target_x); // x-axis, degree
            float p2 = -cal_frame_angle(copter.g2.user_parameters.cam_height.get(), copter.g2.user_parameters.cam_angle_y.get(), tmp_msg._msg_1.content.msg.target_y); // y-axis, degree
            // p2 += copter.g2.user_parameters.cam_pitch_offset.get(); // add offset between cam and uav
            handle_info(p1, p2);
        } else {
            // unhealthy massage
        }
        tmp_msg._msg_1.updated = false;
    }

    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((copter.g2.user_parameters.attack_timeout > 0) && (tnow - _last_ms > (uint32_t)copter.g2.user_parameters.attack_timeout)) {
        // if (_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "valid %ld|%ld", tnow, _last_ms);
        // }
        _valid = false;
        _pitch_filter.reset();
        _yaw_filter.reset();
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

float UCam::cal_frame_angle(float pixel, float angle, float x_in)
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

void UCam::do_cmd_on(bool on) {
    if (on) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Lock ON");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Lock OFF");
    }

    FD_CAM_CMD &tmp_msg = FD_CAM_ptr->get_msg_cam_cmd();

    tmp_msg._msg_1.content.msg.header.head_1 = FD_CAM_CMD::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD_CAM_CMD::PREAMBLE2;
    tmp_msg._msg_1.content.msg.length = 0x10;
    tmp_msg._msg_1.content.msg.frametype = 0x69;
    tmp_msg._msg_1.content.msg.on = (uint8_t)on;
    tmp_msg._msg_1.content.msg.target_x = (int16_t)copter.g2.user_parameters.lock_x;
    tmp_msg._msg_1.content.msg.target_y = (int16_t)copter.g2.user_parameters.lock_y;
    tmp_msg._msg_1.content.msg.type = 0x01;
    tmp_msg._msg_1.content.msg.size = (uint8_t)copter.g2.user_parameters.lock_size;

    tmp_msg.make_sum();
    tmp_msg._msg_1.need_send = true;

    FD_CAM_ptr->write();
}

bool UCam::is_valid() {
    return _valid;
}

void UCam::handle_info(float p1, float p2) {
        // if (!_valid) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "IIvalid %ld|%ld", millis(), _last_ms);
        // }
    _valid = true;
    _last_ms = millis();

    float _roll = copter.ahrs_view->roll;
    float _pitch = copter.ahrs_view->pitch;
    float _yaw = copter.ahrs_view->yaw;
    if (!copter.udelay.get_idx(10-1, _roll, _pitch, _yaw)) {
        _roll = copter.ahrs_view->roll;
        _pitch = copter.ahrs_view->pitch;
        _yaw = copter.ahrs_view->yaw;
    }

    _frotend.bf_info.x = p1; // yaw degree
    _frotend.bf_info.y = p2; // pitch degree

    Matrix3f tmp_m;
    tmp_m.from_euler(_roll, _pitch, 0.0f);

    float dist_z = -tanf(radians(p2));
    float dist_y = tanf(radians(p1));

    Vector3f tmp_input = Vector3f(1.0f,dist_y,dist_z);
    Vector3f tmp_output = tmp_m*tmp_input;

    float angle_pitch = wrap_180(degrees(atan2f(-tmp_output.z, tmp_output.x)));
    float angle_yaw = wrap_180(degrees(atan2f(tmp_output.y, tmp_output.x)));

    _frotend.ef_info.x = wrap_360(angle_yaw + degrees(_yaw));
    _frotend.ef_info.y = angle_pitch;

    _yaw_filter.update(angle_yaw, millis());
    _pitch_filter.update(angle_pitch, millis());

    _frotend.ef_rate_info.x = _yaw_rate_filter.get() + _yaw_filter.slope()*1000.f;
    // _frotend.ef_rate_info.x = _yaw_filter.slope()*1000.f;
    _frotend.ef_rate_info.y = _pitch_filter.slope()*1000.f;

    _frotend.udpate_control_value();

    // _frotend.display_info_p1 = _yaw_rate_filter.get();
    // _frotend.display_info_p2 = _yaw_filter.slope()*1000.f;
    // _frotend.display_info_p3 = _frotend.ef_info.x;
    // _frotend.display_info_p4 = _frotend.ef_info.y;

    _frotend.display_info_p1 = _frotend.bf_info.x;
    _frotend.display_info_p2 = _frotend.bf_info.y;
    _frotend.display_info_p3 = _frotend.ef_info.x;
    _frotend.display_info_p4 = _frotend.ef_info.y;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
}

void UCam::handle_info_test(float p1, float p2) {
    FD_CAM_TARGET &tmp_msg = FD_CAM_ptr->get_msg_cam_target();
    tmp_msg._msg_1.updated = true;
    tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    tmp_msg._msg_1.content.msg.status = 1;
}
