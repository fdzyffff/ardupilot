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

UCam::UCam()
{
    ;
}

// initialise
void UCam::init()
{
    _active = false;
    _cam_state = 0;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    _last_update_ms = 0;
    _target_pitch_rate = 0.0f;
    _target_yaw_rate_cds = 0.0f;
    _target_roll_angle = 0.0f;
    _current_angle_deg = 0.0f;
    _n_count = 0;
    _port = NULL;
    init_port();
}


// clear return path and set home location.  This should be called as part of the arming procedure
void UCam::handle_info(float p1, float p2, float p3, float p4)
{

    copter.Utarget.display_info_p1 = p1;
    copter.Utarget.display_info_p2 = p2;
    copter.Utarget.display_info_p3 = p3;
    copter.Utarget.display_info_p4 = p4;
    copter.Utarget.display_info_new = true;
    copter.Utarget.display_info_count++;
    bool _valid = false;
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 0) {
    // self check fail (2Hz)
        _cam_state = 1;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 1) {
    // self check pass (2Hz)
        _cam_state = 2;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 3) {
    // handle cmd from apm
        _cam_state = 3;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 2) {
    // working
        _cam_state = 4;
    }
    if ((int16_t)p3 == 1 && (int16_t)p4 == 1) {
    // target following
        _cam_state = 5;
        _valid = true;
    }

    if (!_valid) {
        _n_count = 0;
        return;
    }
    raw_info.x = p1;
    raw_info.y = p2;
    Matrix3f tmp_m;
    if (is_zero(copter.g2.user_parameters.fly_roll_factor)) {
        tmp_m.from_euler(0.0f, 0.0f, 0.0f);
    } else {
        tmp_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    }
    Vector3f tmp_input = Vector3f(100.f,p1,-p2);
    Vector3f tmp_output = tmp_m*tmp_input;
    correct_info.x = tmp_output.y;
    correct_info.y = -tmp_output.z;
    _last_update_ms = millis();
    if (!_active) {
        _n_count += 1;
    }
}

int16_t UCam::cam_state() {
    if (_active) {
        return 10;
    }
    return _cam_state;
}

const Vector2f& UCam::get_raw_info() {
    return raw_info;
}

const Vector2f& UCam::get_correct_info() {
    return correct_info;
}


void UCam::init_port()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Cam, 0))) {
        gcs().send_text(MAV_SEVERITY_INFO, "CAM MAV init");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "CAM MAV init FAILED");
    }
}

void UCam::mav_read()
{
    if (get_port() == NULL) {return;}
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    static uint32_t pk0_count = 0;
    static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    //chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    while (get_port()->available()>0) {
        uint8_t temp = get_port()->read();
        if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);
        if (ret == MAVLINK_FRAMING_OK) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t pkt;
                    mavlink_msg_heartbeat_decode(&msg, &pkt);
                    //gcs().send_text(MAV_SEVERITY_INFO, "type=%u autopilot=%u base_mode=0x%x\n", pkt.type, pkt.autopilot, pkt.base_mode);
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    pk1_count++;
                    mavlink_command_long_t packet;
                    mavlink_msg_command_long_decode(&msg, &packet); 
                    switch(packet.command) {
                        case MAV_CMD_USER_1: {
                            handle_info(packet.param1, packet.param2, packet.param3, packet.param4);
                        }
                            break;
                        default:
                            break;
                    }

                    break;
                }

                case MAVLINK_MSG_ID_MY_MICRO_IMAGE:      // MAV ID: 3
                {
                    // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes

                    mavlink_my_micro_image_t packet;
                    mavlink_msg_my_micro_image_decode(&msg, &packet);
                    copter.send_my_micro_image((mavlink_channel_t)0, &packet);
                    break;
                }
                default:
                    break;
            }
        }
    }
    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
        pk0_count = 0;
        pk1_count = 0;
        // pk2_count = 0;
    }
}


void UCam::do_cmd(float p1)
{
    if (get_port() == NULL) {return;}
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    uint16_t len;
    mavlink_command_long_t packet;
    packet.command = MAV_CMD_USER_1;
    packet.param1 = p1;
    packet.param2 = 0.0f;
    packet.param3 = 0.0f;
    packet.param4 = 0.0f;
    packet.param5 = 0.0f;
    packet.param6 = 0.0f;
    packet.param7 = 0.0f;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.confirmation = 0;

    len = mavlink_msg_command_long_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);

    get_port()->write(&msg.magic, 2);
    get_port()->write(&msg.magic+4, 4);
    get_port()->write(&msg.magic+10, len-6);

    // mavlink_heartbeat_t heartbeat = {0};
    // heartbeat.type = 1;
    // heartbeat.autopilot = 2;
    // heartbeat.base_mode = 3;
    // heartbeat.system_status = 4;
    // heartbeat.mavlink_version = 0;
    // heartbeat.custom_mode = 1;

    /*
     save and restore sequence number for chan0, as it is used by
     generated encode functions
    */
    // len = mavlink_msg_heartbeat_encode(5,
    //                                    8,
    //                                    &msg, &heartbeat);

//    gcs().send_text(MAV_SEVERITY_INFO, "ck %d  %d (%d)",(uint8_t)(msg.checksum & 0xFF), (uint8_t)(msg.checksum >> 8), len);


    // get_port()->write(&msg.magic, 2);
    // get_port()->write(&msg.magic+4, 4);
    // get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
}


// update
void UCam::update()
{
    mav_read();
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _n_count = 0;
        _active = false;
    } else if (_n_count > 10) {
        _active = true;
    } else {
        _active = false;
    }

    if ((now - _last_update_ms) > 2000) {
        _cam_state = 0;
        raw_info.x = 0.0f;
        raw_info.y = 0.0f;
    }

    if (!_active) {
        return;
    }
    if (copter.g2.user_parameters.cam_pixel_x.get() < 50.f) {copter.g2.user_parameters.cam_pixel_x.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_pixel_y.get() < 50.f) {copter.g2.user_parameters.cam_pixel_y.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_angle_x.get() < 30.f) {copter.g2.user_parameters.cam_angle_x.set_and_save(30.f);}
    if (copter.g2.user_parameters.cam_angle_y.get() < 30.f) {copter.g2.user_parameters.cam_angle_y.set_and_save(30.f);}
    if (copter.g2.user_parameters.fly_yaw_tc.get() < 0.1f) {copter.g2.user_parameters.fly_yaw_tc.set_and_save(0.1f);}
    update_target_pitch_rate();
    update_target_roll_angle();
    update_target_yaw_rate();
    update_target_track_angle();
}

void UCam::update_target_pitch_rate() {
    float measurement = (get_correct_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y); //-1 to +0.1
    float my_target_pitch_rate = -1.0f*copter.g2.user_parameters.Ucam_pid.update_all(copter.g2.user_parameters.cam_target_y, measurement, false)*copter.g2.user_parameters.fly_pitch_limit.get();
    if (my_target_pitch_rate > 0.0f) {
        my_target_pitch_rate *= 1.0f;
    }
    _target_pitch_rate = my_target_pitch_rate ;
}

void UCam::update_target_roll_angle() {
    float info_x = get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    _target_roll_angle = copter.g2.user_parameters.fly_roll_limit*info_x*copter.g2.user_parameters.fly_roll_factor ;
}

void UCam::update_target_yaw_rate() {
    float info_x = get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    float x_angle = copter.g2.user_parameters.cam_angle_x;
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = 100.f * (x_angle * info_x / yaw_rate_tc);
    _target_yaw_rate_cds = yaw_rate_cds;
}

void UCam::update_target_track_angle() {
    _current_angle_deg = -degrees(copter.ahrs_view->pitch) - (copter.g2.user_parameters.cam_pitch_offset)*0.01f-degrees(atanf((get_correct_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y)*tanf(0.5f*radians(copter.g2.user_parameters.cam_angle_y))));
}
