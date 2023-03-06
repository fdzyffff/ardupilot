#include "Plane.h"

void Plane::HB1_init() {
    uart_ins.init();
    uart_ins.get_msg_ins2apm().set_enable();
}

void Plane::HB1_run() {
    uart_ins.read();
    HB1_handle_msg();
    uart_ins.write();
}

void Plane::HB1_handle_msg() {

    HB1_ins2apm &tmp_msg = uart_ins.get_msg_ins2apm();

    if (tmp_msg._msg_1.updated) {
        // gcs().send_text(MAV_SEVERITY_INFO, "length:%d", tmp_msg._msg_1.content.msg.length);
        // gcs().send_text(MAV_SEVERITY_INFO, "id:%d", tmp_msg._msg_1.content.msg.id);
        // gcs().send_text(MAV_SEVERITY_INFO, "counter:%d", tmp_msg._msg_1.content.msg.counter);
        // gcs().send_text(MAV_SEVERITY_INFO, "state:%d", tmp_msg._msg_1.content.msg.state);
        // gcs().send_text(MAV_SEVERITY_INFO, "pitch:%f", tmp_msg._msg_1.content.msg.pitch);
        // gcs().send_text(MAV_SEVERITY_INFO, "roll:%f", tmp_msg._msg_1.content.msg.roll);
        // gcs().send_text(MAV_SEVERITY_INFO, "yaw:%f", tmp_msg._msg_1.content.msg.yaw);
        // gcs().send_text(MAV_SEVERITY_INFO, "yaw_gps:%f", tmp_msg._msg_1.content.msg.yaw_gps);
        // gcs().send_text(MAV_SEVERITY_INFO, "pitch_rate:%f", tmp_msg._msg_1.content.msg.pitch_rate);
        // gcs().send_text(MAV_SEVERITY_INFO, "roll_rate:%f", tmp_msg._msg_1.content.msg.roll_rate);
        // gcs().send_text(MAV_SEVERITY_INFO, "yaw_rate:%f", tmp_msg._msg_1.content.msg.yaw_rate);
        // gcs().send_text(MAV_SEVERITY_INFO, "longitude:%d", tmp_msg._msg_1.content.msg.longitude);
        // gcs().send_text(MAV_SEVERITY_INFO, "latitude:%d", tmp_msg._msg_1.content.msg.latitude);
        // gcs().send_text(MAV_SEVERITY_INFO, "alt_baro:%d", tmp_msg._msg_1.content.msg.alt_baro);
        // gcs().send_text(MAV_SEVERITY_INFO, "alt_gps:%d", tmp_msg._msg_1.content.msg.alt_gps);
        // gcs().send_text(MAV_SEVERITY_INFO, "alt_ekf:%d", tmp_msg._msg_1.content.msg.alt_ekf);
        // gcs().send_text(MAV_SEVERITY_INFO, "vel_n:%f", tmp_msg._msg_1.content.msg.vel_n);
        // gcs().send_text(MAV_SEVERITY_INFO, "vel_e:%f", tmp_msg._msg_1.content.msg.vel_e);
        // gcs().send_text(MAV_SEVERITY_INFO, "vel_d:%f", tmp_msg._msg_1.content.msg.vel_d);
        // gcs().send_text(MAV_SEVERITY_INFO, "airspeed:%f", tmp_msg._msg_1.content.msg.airspeed);
        // gcs().send_text(MAV_SEVERITY_INFO, "acc_n:%f", tmp_msg._msg_1.content.msg.acc_n);
        // gcs().send_text(MAV_SEVERITY_INFO, "acc_e:%f", tmp_msg._msg_1.content.msg.acc_e);
        // gcs().send_text(MAV_SEVERITY_INFO, "acc_d:%f", tmp_msg._msg_1.content.msg.acc_d);
        // gcs().send_text(MAV_SEVERITY_INFO, "satellite_count:%d", tmp_msg._msg_1.content.msg.satellite_count);
        // gcs().send_text(MAV_SEVERITY_INFO, "hdop:%d", tmp_msg._msg_1.content.msg.hdop);
        // gcs().send_text(MAV_SEVERITY_INFO, "vdop:%d", tmp_msg._msg_1.content.msg.vdop);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_status:%d", tmp_msg._msg_1.content.msg.gps_status);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_hh:%d", tmp_msg._msg_1.content.msg.gps_hh);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_mm:%d", tmp_msg._msg_1.content.msg.gps_mm);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_ss:%d", tmp_msg._msg_1.content.msg.gps_ss);
        // gcs().send_text(MAV_SEVERITY_INFO, "temperature:%d", tmp_msg._msg_1.content.msg.temperature);
        // gcs().send_text(MAV_SEVERITY_INFO, "hdt:%d", tmp_msg._msg_1.content.msg.hdt);
        // gcs().send_text(MAV_SEVERITY_INFO, "hdt_dev:%d", tmp_msg._msg_1.content.msg.hdt_dev);
        // gcs().send_text(MAV_SEVERITY_INFO, "sensor_used:%d", tmp_msg._msg_1.content.msg.sensor_used);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps0_dt:%d", tmp_msg._msg_1.content.msg.gps0_dt);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps1_dt:%d", tmp_msg._msg_1.content.msg.gps1_dt);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_vn:%f", tmp_msg._msg_1.content.msg.gps_vn);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_ve:%f", tmp_msg._msg_1.content.msg.gps_ve);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_vd:%f", tmp_msg._msg_1.content.msg.gps_vd);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_ms:%d", tmp_msg._msg_1.content.msg.gps_ms);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_day:%d", tmp_msg._msg_1.content.msg.gps_day);
        // gcs().send_text(MAV_SEVERITY_INFO, "gps_week:%d", tmp_msg._msg_1.content.msg.gps_week);
        // gcs().send_text(MAV_SEVERITY_INFO, "ahrs_state:%d", tmp_msg._msg_1.content.msg.ahrs_state);
        // gcs().send_text(MAV_SEVERITY_INFO, "sum_check:%x", tmp_msg._msg_1.content.msg.sum_check);
        tmp_msg._msg_1.updated = false;

        AP_ExternalAHRS::gps_data_message_t gps {
            gps_week: 0,
            ms_tow: millis(),
            fix_type: (uint8_t) ((tmp_msg._msg_1.content.msg.ahrs_state>0?)AP_GPS::GPS_FIX_TYPE_3D_FIX:AP_GPS::GPS_FIX_TYPE_NO_FIX),
            satellites_in_view: 10,

            horizontal_pos_accuracy: 1.0f,
            vertical_pos_accuracy: 1.0f,
            horizontal_vel_accuracy: 1.0f,

            hdop: 1.0f,
            vdop: 1.0f,

            longitude: tmp_msg._msg_1.content.msg.longitude,
            latitude: tmp_msg._msg_1.content.msg.latitude,
            msl_altitude: tmp_msg._msg_1.content.msg.alt_ekf,

            ned_vel_north: tmp_msg._msg_1.content.msg.gps_vn,
            ned_vel_east: tmp_msg._msg_1.content.msg.gps_ve,
            ned_vel_down: tmp_msg._msg_1.content.msg.gps_vd,
        };
        AP::gps().handle_external(gps);
    }
}
