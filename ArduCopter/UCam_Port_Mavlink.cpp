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

UCam_Port_Mavlink::UCam_Port_Mavlink(UCam &frotend_in):
    UCam_Port(frotend_in)
{
    ;
}

void UCam_Port_Mavlink::port_read() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();
    static uint32_t pk0_count = 0;
    static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    //chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    while (_frotend.get_port()->available()>0) {
        uint8_t temp = _frotend.get_port()->read();
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
                            handle_info(&packet);
                            // copter.gcs().send_text(MAV_SEVERITY_WARNING, "CAM USER1");
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

void UCam_Port_Mavlink::handle_info(const mavlink_command_long_t* packet)
{
    float p1 = packet->param1;
    float p2 = packet->param2;
    float p3 = packet->param3;
    float p4 = packet->param4;
    float p5 = packet->param5;
    float p6 = packet->param6;
    float p7 = packet->param7;

    _frotend.display_info_p1 = p1;
    _frotend.display_info_p2 = p2;
    _frotend.display_info_p3 = p3;
    _frotend.display_info_p4 = p4;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
    bool _valid = false;
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 0 && (int16_t)p5 == 0) {
    // self check fail (2Hz)
        _frotend._cam_state = 1;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 1) {
    // self check pass (2Hz)
        _frotend._cam_state = 2;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 3) {
    // handle cmd from apm
        _frotend._cam_state = 3;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 2) {
    // working
        _frotend._cam_state = 4;
    }
    if ((int16_t)p3 == 1 && (int16_t)p4 == 1) {
    // target following
        _frotend._cam_state = 5;
        _valid = true;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 0 && (int16_t)p5 == 13) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "Para [%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f]", p1, p2, p3, p4, p5, p6, p7);
        mavlink_command_long_t new_packet;
        new_packet.param1 = 11.0f;
        new_packet.param2 = 0.0f;
        new_packet.param3 = p6;
        new_packet.param4 = p7;
        new_packet.param5 = 0.0f;
        new_packet.param6 = 0.0f;
        new_packet.param7 = 0.0f;
        copter.send_my_command_long((mavlink_channel_t)0, &new_packet);
    }

    if (!_valid) {
        _frotend._n_count = 0;
        return;
    }

    float dt = (float)(millis() - _frotend._last_update_ms)*1.0e-3f;
    if (dt > 1.0f) {dt = 1.0f;}
    if (dt < 0.01f) {dt = 0.01f;} // sainty check, cam info will be at around 20Hz, conrresponding to 0.05s.
    
    _frotend.raw_info.x = p1;
    _frotend.raw_info.y = p2;
    Matrix3f tmp_m;
    if (is_zero(copter.g2.user_parameters.fly_roll_factor)) {
        tmp_m.from_euler(0.0f, 0.0f, 0.0f);
    } else {
        tmp_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    }
    Vector3f tmp_input = Vector3f(100.f,p1,-p2);
    Vector3f tmp_output = tmp_m*tmp_input;
    _frotend._cam_filter.apply(tmp_output, dt);
    _frotend.correct_info.x = _frotend._cam_filter.get().y;
    _frotend.correct_info.y = -_frotend._cam_filter.get().z;
    _frotend._last_update_ms = millis();
    if (!_frotend._active) {
        _frotend._n_count += 1;
    }
    _frotend.update_value(dt);
}
void UCam_Port_Mavlink::do_cmd(float p1, float p2, float p3, float p4) {
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
    packet.param2 = p2;
    packet.param3 = p3;
    packet.param4 = p4;
    packet.param5 = 0.0f;
    packet.param6 = 0.0f;
    packet.param7 = 0.0f;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.confirmation = 0;

    copter.gcs().send_text(MAV_SEVERITY_WARNING, "SEND [%0.2f,%0.2f,%0.2f,%0.2f]", p1, p2, p3, p4);

    len = mavlink_msg_command_long_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);

    _frotend.get_port()->write(&msg.magic, 2);
    _frotend.get_port()->write(&msg.magic+4, 4);
    _frotend.get_port()->write(&msg.magic+10, len-6);

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


    // _frotend.get_port()->write(&msg.magic, 2);
    // _frotend.get_port()->write(&msg.magic+4, 4);
    // _frotend.get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
}
