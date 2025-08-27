#include "Copter.h"

UMission::UMission()
{

}

// initialise
void UMission::init()
{
    _valid = false;
    _uart_control.init();
    _uart_control.get_msg_status().set_enable();
    _uart_control.get_msg_control().set_enable();
    _last_ms = 0;
    _last_log_ms = 0;
    _last_mav_ms = 0;
    _last_rate_ms = 0;
    _msg_count = 0;

    if (_uart_control.initialized()) {
        gcs().send_text(MAV_SEVERITY_INFO, "[Danji] INIT");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[Danji] INIT FAIL");
        gcs().send_text(MAV_SEVERITY_INFO, "[Danji] INIT FAIL");
        gcs().send_text(MAV_SEVERITY_INFO, "[Danji] INIT FAIL");
    }


    // // start calls to loop in separate thread
    // if (!hal.scheduler->thread_create(
    //         FUNCTOR_BIND_MEMBER(&UMission::send_raw_imu_loop, void), "IMURAW", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
    //         gcs().send_text(MAV_SEVERITY_INFO, "IMURAW: couldn't create thread\n\r");
    // } else {
    //     gcs().send_text(MAV_SEVERITY_INFO, "IMURAW: create thread\n\r");
    // }
}

// called at 400 Hz
void UMission::update()
{
    update_uart_read();
    update_uart_send();
    update_valid();
    update_mav();
    update_rate();
    if (copter.g2.user_parameters.log_test.get() == 1) {
        update_log();
        // if (_uart_control.initialized() && _uart_control.get_port() != nullptr) {
        //     _uart_control.get_msg_control()._msg_1.updated = true;
        //     handle_msg_control();
        // }

    }
}

void UMission::update_uart_read()
{
    if (_uart_control.initialized() && _uart_control.get_port() != nullptr) {
        while (_uart_control.get_port()->available() > 0) {
            uint8_t temp = _uart_control.get_port()->read();
            _uart_control.read(temp);
            if (_uart_control.get_msg_control()._msg_1.updated) {
                handle_msg_control();
            }
        }
    }
}

void UMission::update_uart_send()
{
    // send msg status
    send_status();
}

void UMission::update_valid()
{
    if (_last_ms == 0) {
        _valid = false;
        return;
    }

    if (millis() - _last_ms > 2000) {
        if (_valid) {
            _valid = false;
            gcs().send_text(MAV_SEVERITY_INFO, "[Danji] Lost control");
        }
    } else {
        if (!_valid) {
            _valid = true;
            gcs().send_text(MAV_SEVERITY_INFO, "[Danji] Get control");
        }
    }
}

void UMission::handle_msg_control()
{
    FD1_msg_control &tmp_msg = _uart_control.get_msg_control();
    if (tmp_msg._msg_1.updated) {
        _control_corr_bfy = (float)(tmp_msg._msg_1.content.msg.corr_bfy)*0.01f;
        _control_corr_bfz = (float)(tmp_msg._msg_1.content.msg.corr_bfz)*0.01f;
        tmp_msg._msg_1.updated = false;
        _last_ms = millis();
        _msg_count++;
        update_log();
    }

    static uint32_t _last_print_ms = millis();
    if (millis() - _last_print_ms > 2000) {
        _last_print_ms = millis();
        if (copter.g2.user_parameters.print.get() > 0) {
            gcs().send_text(MAV_SEVERITY_INFO, " control_time %f", ((float)tmp_msg._msg_1.content.msg.control_time)*0.001f);
            gcs().send_text(MAV_SEVERITY_INFO, " control_id %d", (tmp_msg._msg_1.content.msg.control_id));
            gcs().send_text(MAV_SEVERITY_INFO, " field_pitch %f", ((float)tmp_msg._msg_1.content.msg.field_pitch)*0.001f);
            gcs().send_text(MAV_SEVERITY_INFO, " field_yaw %f", ((float)tmp_msg._msg_1.content.msg.field_yaw)*0.002f);
            gcs().send_text(MAV_SEVERITY_INFO, " target_id %d", (tmp_msg._msg_1.content.msg.target_id));
            gcs().send_text(MAV_SEVERITY_INFO, " target_pitch %f", ((float)tmp_msg._msg_1.content.msg.target_pitch)*0.001f);
            gcs().send_text(MAV_SEVERITY_INFO, " target_yaw %f", ((float)tmp_msg._msg_1.content.msg.target_yaw)*0.002f);
            gcs().send_text(MAV_SEVERITY_INFO, " lng %f", ((float)tmp_msg._msg_1.content.msg.lng)*0.000001f);
            gcs().send_text(MAV_SEVERITY_INFO, " lat %f", ((float)tmp_msg._msg_1.content.msg.lat)*0.000001f);
            gcs().send_text(MAV_SEVERITY_INFO, " alt %f", ((float)tmp_msg._msg_1.content.msg.alt)*0.1f);
            gcs().send_text(MAV_SEVERITY_INFO, " launch_lng %f", ((float)tmp_msg._msg_1.content.msg.launch_lng)*0.000001f);
            gcs().send_text(MAV_SEVERITY_INFO, " launch_lat %f", ((float)tmp_msg._msg_1.content.msg.launch_lat)*0.000001f);
            gcs().send_text(MAV_SEVERITY_INFO, " launch_alt %f", ((float)tmp_msg._msg_1.content.msg.launch_alt)*0.1f);
            gcs().send_text(MAV_SEVERITY_INFO, " R %d", (tmp_msg._msg_1.content.msg.R));
            gcs().send_text(MAV_SEVERITY_INFO, " djy %f", ((float)tmp_msg._msg_1.content.msg.djy)*0.001f);
            gcs().send_text(MAV_SEVERITY_INFO, " djz %f", ((float)tmp_msg._msg_1.content.msg.djz)*0.002f);
            gcs().send_text(MAV_SEVERITY_INFO, " corr_bfy %f", ((float)tmp_msg._msg_1.content.msg.corr_bfy)*0.01f);
            gcs().send_text(MAV_SEVERITY_INFO, " corr_bfz %f", ((float)tmp_msg._msg_1.content.msg.corr_bfz)*0.01f);
        }
    }
}

void UMission::send_status()
{
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms < 5) {return;}
    _last_send_ms = millis();

    FD1_msg_status &tmp_msg = _uart_control.get_msg_status();

    //uint32_t
    tmp_msg._msg_1.content.msg.sys_time = millis();
    //uint8_t
    tmp_msg._msg_1.content.msg.gps_ok = copter.position_ok()?2:0;
    //int32_t
    tmp_msg._msg_1.content.msg.lng = copter.position_ok()? copter.current_loc.lng/10 : 0;
    //int32_t
    tmp_msg._msg_1.content.msg.lat = copter.position_ok()? copter.current_loc.lat/10 : 0;

    int32_t tmp_alt = 0;
    if (copter.position_ok()) {
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            tmp_alt = tmp_alt/10;
        }
    }

    if (tmp_alt < 0) {
        tmp_alt = 0;
    }
    //uint16_t
    tmp_msg._msg_1.content.msg.alt = (uint16_t)(tmp_alt);

    int32_t home_lng = 0;
    int32_t home_lat = 0;
    int32_t home_alt = 0;
    if (copter.ahrs.home_is_set()) {
        home_lng = copter.ahrs.get_home().lng/10;
        home_lat = copter.ahrs.get_home().lat/10;
        if (copter.ahrs.get_home().get_alt_cm(Location::AltFrame::ABSOLUTE, home_alt)) {
            home_alt = home_alt/10;
                if (home_alt < 0) {
                    home_alt = 0;
                }
        }
    } else if (copter.position_ok()) {
        home_lng = copter.current_loc.lng/10;
        home_lat = copter.current_loc.lat/10;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, home_alt)) {
            home_alt = home_alt/10;
        }
    }
    //int32_t
    tmp_msg._msg_1.content.msg.launch_lng = home_lng;
    //int32_t
    tmp_msg._msg_1.content.msg.launch_lat = home_lat;
    //uint16_t
    tmp_msg._msg_1.content.msg.launch_alt = (uint16_t)home_alt;

    tmp_msg.make_sum();

    tmp_msg._msg_1.need_send = true;

    _uart_control.write();
}

// for test purpose
void UMission::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                {
                    _last_ms = millis();
                    _control_corr_bfy = packet.param1*0.01f;
                    _control_corr_bfz = packet.param2*0.01f;
                }
                break;
            default:
                break;
        }
    }
}

void UMission::update_rate()
{
    if (millis() - _last_rate_ms < 1000) {return;}
    float dt = 0.001f * (float)(millis() - _last_rate_ms);
    float rate = (float)_msg_count/dt;
    if (copter.g2.user_parameters.print.get() > 0) {
        gcs().send_text(MAV_SEVERITY_INFO, " [Rate] %f/s", rate);
    }
    _msg_count = 0;
    _last_rate_ms = millis();

    AP::logger().WriteStreaming("UDN3",
                                "TimeUS,rate",
                                "s-",
                                "F-",
                                "Qf",
                                AP_HAL::micros64(),
                                rate);
}

void UMission::update_log()
{
    if ((copter.g2.user_parameters.log_raw.get() == 0) && (millis() - _last_log_ms < 100)) {return;}
    _last_log_ms = millis();

    FD1_msg_control &tmp_msg = _uart_control.get_msg_control();

    // if (!copter.motors->armed()) {return;}
    AP::logger().WriteStreaming("UDN1",
                                "TimeUS,ctim,cid,fpth,fyaw,tid,tpth,tyaw,lng,lat,alt",
                                "s----------",
                                "F----------",
                                "QHBhhBhhiiH",
                                AP_HAL::micros64(),
                                tmp_msg._msg_1.content.msg.control_time,
                                tmp_msg._msg_1.content.msg.control_id,
                                tmp_msg._msg_1.content.msg.field_pitch,
                                tmp_msg._msg_1.content.msg.field_yaw,
                                tmp_msg._msg_1.content.msg.target_id,
                                tmp_msg._msg_1.content.msg.target_pitch,
                                tmp_msg._msg_1.content.msg.target_yaw,
                                tmp_msg._msg_1.content.msg.lng,
                                tmp_msg._msg_1.content.msg.lat,
                                tmp_msg._msg_1.content.msg.alt);
    AP::logger().WriteStreaming("UDN2",
                                "TimeUS,hlng,hlat,halt,R,djy,djz,cdjy,cdjz",
                                "s--------",
                                "F--------",
                                "QiiHhhhhh",
                                AP_HAL::micros64(),
                                tmp_msg._msg_1.content.msg.launch_lng,
                                tmp_msg._msg_1.content.msg.launch_lat,
                                tmp_msg._msg_1.content.msg.launch_alt,
                                tmp_msg._msg_1.content.msg.R,
                                tmp_msg._msg_1.content.msg.djy,
                                tmp_msg._msg_1.content.msg.djz,
                                tmp_msg._msg_1.content.msg.corr_bfy,
                                tmp_msg._msg_1.content.msg.corr_bfz);
    // AP::logger().WriteStreaming("UDNJ",
    //                             "TimeUS,ct,ci,fp,fy,ti,tp,ty,lng,lat,alt,R,djy,djz,cdjy,cdjz",
    //                             "s---------------",
    //                             "F---------------",
    //                             "QHBhhBhhiiHhhhhh",
    //                             AP_HAL::micros64(),
    //                             tmp_msg._msg_1.content.msg.control_time,
    //                             tmp_msg._msg_1.content.msg.control_id,
    //                             tmp_msg._msg_1.content.msg.field_pitch,
    //                             tmp_msg._msg_1.content.msg.field_yaw,
    //                             tmp_msg._msg_1.content.msg.target_id,
    //                             tmp_msg._msg_1.content.msg.target_pitch,
    //                             tmp_msg._msg_1.content.msg.target_yaw,
    //                             tmp_msg._msg_1.content.msg.lng,
    //                             tmp_msg._msg_1.content.msg.lat,
    //                             tmp_msg._msg_1.content.msg.alt,
    //                             tmp_msg._msg_1.content.msg.R,
    //                             tmp_msg._msg_1.content.msg.djy,
    //                             tmp_msg._msg_1.content.msg.djz,
    //                             tmp_msg._msg_1.content.msg.corr_bfy,
    //                             tmp_msg._msg_1.content.msg.corr_bfz);
}

void UMission::update_mav()
{
    if (millis() - _last_mav_ms < 200) {return;}
    _last_mav_ms = millis();
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_command_long_send(
                    channel,
                    0,
                    0,
                    MAV_CMD_USER_1,
                    0,
                    _control_corr_bfy,
                    _control_corr_bfz,
                    0, 0, 0, 0, 0);
            }
        }
    }
}



void UMission::send_raw_imu_loop() {
    // hal.scheduler->delay(3000);
    gcs().send_text(MAV_SEVERITY_INFO, "LOOP IMURAW Start");
    while (true) {
        send_raw_imu();
    }
}


void UMission::send_raw_imu()
{
    static uint32_t _last_l_ms = millis();
    static int16_t count = 0;

    float dt = (float)(millis() - _last_l_ms)*0.001f;
    if (dt > 1.0f) {
        // gcs().send_text(MAV_SEVERITY_INFO, "LOOP IMURAW %d", count);
        _last_l_ms = millis();
        float imu_rate = ((float)count)/dt;
        count = 0;
        AP::logger().WriteStreaming("UIMU",
                                    "TimeUS,rate",
                                    "s-",
                                    "F-",
                                    "Qf",
                                    AP_HAL::micros64(),
                                    (float)imu_rate);
    }

    count++;
    hal.scheduler->delay_microseconds(5000);
}
