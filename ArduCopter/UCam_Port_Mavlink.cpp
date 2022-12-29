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
                            _frotend.handle_info(&packet);
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
