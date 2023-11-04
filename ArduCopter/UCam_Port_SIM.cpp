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

UCam_Port_SIM::UCam_Port_SIM(UCam &frotend_in):
    UCam_Port(frotend_in)
{
    ;
}

void UCam_Port_SIM::port_read() {
    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // // static uint32_t pk2_count = 0;
    // mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    // uint8_t saved_seq = chan0_status->current_tx_seq;
    // uint8_t saved_flags = chan0_status->flags;
    // //chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    // while (_frotend.get_port()->available()>0) {
    //     uint8_t temp = _frotend.get_port()->read();
    //     if (temp == 0xFE) pk0_count++;
    //     mavlink_message_t msg;
    //     mavlink_status_t status;
    //     uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);
    //     if (ret == MAVLINK_FRAMING_OK) {
    //         switch (msg.msgid) {
    //             case MAVLINK_MSG_ID_HEARTBEAT: {
    //                 mavlink_heartbeat_t pkt;
    //                 mavlink_msg_heartbeat_decode(&msg, &pkt);
    //                 //gcs().send_text(MAV_SEVERITY_INFO, "type=%u autopilot=%u base_mode=0x%x\n", pkt.type, pkt.autopilot, pkt.base_mode);
    //                 break;
    //             }
    //             case MAVLINK_MSG_ID_COMMAND_LONG: {
    //                 pk1_count++;
    //                 mavlink_command_long_t packet;
    //                 mavlink_msg_command_long_decode(&msg, &packet); 
    //                 switch(packet.command) {
    //                     case MAV_CMD_USER_1: {
    //                         _frotend.handle_info(&packet);
    //                         // copter.gcs().send_text(MAV_SEVERITY_WARNING, "CAM USER1");
    //                     }
    //                         break;
    //                     default:
    //                         break;
    //                 }

    //                 break;
    //             }

    //             case MAVLINK_MSG_ID_MY_MICRO_IMAGE:      // MAV ID: 3
    //             {
    //                 // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes

    //                 mavlink_my_micro_image_t packet;
    //                 mavlink_msg_my_micro_image_decode(&msg, &packet);
    //                 copter.send_my_micro_image((mavlink_channel_t)0, &packet);
    //                 break;
    //             }
    //             default:
    //                 break;
    //         }
    //     }
    // }
    // chan0_status->current_tx_seq = saved_seq;
    // chan0_status->flags = saved_flags;
    // if (tnow - last_update_ms > 1000) {
    //     //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
    //     last_update_ms = tnow;
    //     pk0_count = 0;
    //     pk1_count = 0;
    //     // pk2_count = 0;
    // }
    _frotend.target_pos_SIM.x = copter.g2.user_parameters.target_sim_x.get();
    _frotend.target_pos_SIM.y = copter.g2.user_parameters.target_sim_y.get();
    _frotend.target_pos_SIM.z = copter.g2.user_parameters.target_sim_z.get();
    // if (copter.mocap_stat.n_count == 0) {
        _frotend.current_pos_SIM.x = copter.inertial_nav.get_position_xy_cm().x*0.01f; //meter
        _frotend.current_pos_SIM.y = copter.inertial_nav.get_position_xy_cm().y*0.01f; //meter
        _frotend.current_pos_SIM.z = copter.inertial_nav.get_position_z_up_cm()*0.01f; //meter
    // }
    handle_info();
}

void UCam_Port_SIM::handle_info() {
    _frotend.display_info_p1 = _frotend.target_pos_SIM.x;
    _frotend.display_info_p2 = _frotend.target_pos_SIM.y;
    _frotend.display_info_p3 = _frotend.target_pos_SIM.z;
    _frotend.display_info_p4 = 11;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;

    _frotend._n_count = 20;

    float dt = (float)(millis() - _frotend._last_update_ms)*1.0e-3f;
    if (dt > 1.0f) {dt = 1.0f;}
    if (dt < 0.01f) {dt = 0.01f;} // sainty check, cam info will be at around 20Hz, conrresponding to 0.05s.
    
    _frotend._last_update_ms = millis();
    if (!_frotend._active) {
        _frotend._n_count += 1;
    }
    _frotend.update_value_SIM(dt);
}

void UCam_Port_SIM::do_cmd(float p1, float p2, float p3, float p4) {
    return;
}
