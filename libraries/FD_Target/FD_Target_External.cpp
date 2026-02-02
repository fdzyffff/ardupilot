#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_External::var_info[] = {

    AP_GROUPINFO("TOUT",   0, FD_Target_External, target_timeout,        2000),

    AP_GROUPEND
};

FD_Target_External::FD_Target_External()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_External::init() {
    _last_ms = 0;
    _valid = false;
    set_type(0);
    _port = nullptr;
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MISSION, 0);
    if (_port == nullptr) {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "Uart External init");
    return true;
}

void FD_Target_External::update() {
    if (get_port() == nullptr) {
        _valid = false;
        return;
    }

        // ::printf("get_port()->available(): %d\n", int(get_port()->available()));
    for (uint32_t i_xx = 0; i_xx < 256; i_xx++) {
        if (get_port()->available()<=0) {
            break;
        }
        uint8_t temp = get_port()->read();
        uart_msg_LS_control.parse(temp);
        if (uart_msg_LS_control._msg_1.updated) {
            uart_msg_LS_control._msg_1.updated = false;


            if (uart_msg_LS_control._msg_1.content.msg.control_type == 0x02) {
                if (isnan(uart_msg_LS_control._msg_1.content.msg.cmd_speed)) {
                    uart_msg_LS_control._msg_1.content.msg.cmd_speed = 0.0f;
                }
                if (isnan(uart_msg_LS_control._msg_1.content.msg.cmd_pitch)) {
                    uart_msg_LS_control._msg_1.content.msg.cmd_pitch = 0.0f;
                }
                if (isnan(uart_msg_LS_control._msg_1.content.msg.cmd_roll)) {
                    uart_msg_LS_control._msg_1.content.msg.cmd_roll = 0.0f;
                }
                _target_speed = uart_msg_LS_control._msg_1.content.msg.cmd_speed;
                _target_pitch = uart_msg_LS_control._msg_1.content.msg.cmd_pitch;
                _target_roll = uart_msg_LS_control._msg_1.content.msg.cmd_roll;
                _last_ms = millis();
                _valid = true;
            }

                // float cmd_speed = uart_msg_LS_control._msg_1.content.msg.cmd_speed;
                // float cmd_pitch = uart_msg_LS_control._msg_1.content.msg.cmd_pitch;
                // float cmd_roll = uart_msg_LS_control._msg_1.content.msg.cmd_roll;
    //         gcs().send_text(MAV_SEVERITY_INFO, "cmd_speed: %f, pitch: %f, roll: %f", cmd_speed, cmd_pitch, cmd_roll);
        // ::printf("cmd_speed: %f, pitch: %f, roll: %f\n", cmd_speed, cmd_pitch, cmd_roll);
        }
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
    }

    pack_status();
}

void FD_Target_External::handle_info_test(float p1, float p2) {
    handle_info(p1, p2);
    // FD_K230_TARGET &tmp_msg = FD_K230_ptr->get_msg_cam_target();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2);
    // tmp_msg._msg_1.content.msg.status = 1;
}

void FD_Target_External::handle_msg(const mavlink_message_t &msg)
{
    // if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    //     // decode packet
    //     // decode packet
    //     mavlink_command_long_t packet;
    //     mavlink_msg_command_long_decode(&msg, &packet);
    //     switch(packet.command) {
    //         case MAV_CMD_USER_1:
    //             gcs().send_text(MAV_SEVERITY_WARNING, "Target External Test, void");
    //             // handle_info_test(packet.param1, packet.param2);
    //             break;
    //         default:
    //             break;
    //     }
    // }
}

void FD_Target_External::set_target_angle(float gimbal_yaw, float gimbal_pitch)
{
    _gimbal_yaw = gimbal_yaw;
    _gimbal_pitch = gimbal_pitch;
}

void FD_Target_External::set_target_loc(Location& loc_in)
{
    _target_loc = loc_in;
}

void FD_Target_External::pack_status()
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return;
    }

    Vector3f pos_ned;
    if ( !AP::ahrs().get_relative_position_NED_home(pos_ned) ) {
        pos_ned.zero();
    }
    Vector3f vel_ned;
    if ( !AP::ahrs().get_velocity_NED(vel_ned) ) {
        vel_ned.zero();
    }


    double target_lng = 0.0;
    double target_lat = 0.0;
    float target_alt = 0.0;
    if (get_target_loc().lng != 0 && get_target_loc().lat != 0) {
        target_lng = ((double)get_target_loc().lng) * 1e-7;
        target_lat = ((double)get_target_loc().lat) * 1e-7;
        int32_t tmp_alt;
        if (get_target_loc().get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            target_alt = ((float)tmp_alt) * 0.01f;
        }
    }


    double current_lng = 0.0;
    double current_lat = 0.0;
    float current_alt = 0.0;
    if (current_loc.lng != 0 && current_loc.lat != 0) {
        current_lng = ((double)current_loc.lng) * 1e-7;
        current_lat = ((double)current_loc.lat) * 1e-7;
        int32_t tmp_alt;
        current_lng = ((double)current_loc.lng) * 1e-7;
        if (current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, tmp_alt)) {
            current_alt = ((float)tmp_alt) * 0.01f;
        }
    }

    float airspeed = 0.0f;
    if (AP::ahrs().airspeed_estimate(airspeed)) {
        ;
    }
    uart_msg_LS_status._msg_1.content.msg.run_time = ((float)AP_HAL::millis()) * 0.001f;
    uart_msg_LS_status._msg_1.content.msg.gimbal_pitch = _gimbal_pitch;
    uart_msg_LS_status._msg_1.content.msg.gimbal_yaw = _gimbal_yaw;
    uart_msg_LS_status._msg_1.content.msg.target_x = 0;
    uart_msg_LS_status._msg_1.content.msg.target_y = 0;
    uart_msg_LS_status._msg_1.content.msg.target_lng = target_lng;
    uart_msg_LS_status._msg_1.content.msg.target_lat = target_lat;
    uart_msg_LS_status._msg_1.content.msg.target_alt = target_alt;
    uart_msg_LS_status._msg_1.content.msg.current_lng = current_lng;
    uart_msg_LS_status._msg_1.content.msg.current_lat = current_lat;
    uart_msg_LS_status._msg_1.content.msg.vel_n = vel_ned.x;
    uart_msg_LS_status._msg_1.content.msg.vel_e = vel_ned.y;
    uart_msg_LS_status._msg_1.content.msg.vel_d = vel_ned.z;
    uart_msg_LS_status._msg_1.content.msg.roll = degrees(AP::ahrs().get_roll());
    uart_msg_LS_status._msg_1.content.msg.pitch = degrees(AP::ahrs().get_pitch());
    uart_msg_LS_status._msg_1.content.msg.yaw = degrees(AP::ahrs().get_yaw());
    uart_msg_LS_status._msg_1.content.msg.air_speed = airspeed;
    uart_msg_LS_status._msg_1.content.msg.yaw_rate = degrees(AP::ahrs().get_yaw_rate_earth());
    uart_msg_LS_status._msg_1.content.msg.pos_x = pos_ned.x;
    uart_msg_LS_status._msg_1.content.msg.pos_y = pos_ned.y;
    uart_msg_LS_status._msg_1.content.msg.pos_z = pos_ned.z;
    uart_msg_LS_status._msg_1.content.msg.current_alt = current_alt;

    uart_msg_LS_status.make_sum();
    uart_msg_LS_status.swap_message();

    if (get_port() != nullptr) {
        get_port()->write(uart_msg_LS_status._msg_1.content.data, sizeof(uart_msg_LS_status._msg_1.content.data));
        // get_port()->write(0xBE);
    }
}