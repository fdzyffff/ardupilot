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

// Convenience macros //////////////////////////////////////////////////////////
//

UCam_Port_ASCII::UCam_Port_ASCII(UCam &frotend_in):
    UCam_Port(frotend_in)
{
    ;
}

void UCam_Port_ASCII::port_read() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();

    while (_frotend.get_port()->available()>0) {
        uint8_t temp = _frotend.get_port()->read();
        switch (temp) {
            case '=':
                if (detect_sentence()) {
                    _line_state = 1;
                    _term_offset = 0;
                    _term_number = 0;
                } else {
                    _line_state = 0;
                    _term_offset = 0;
                    _term_number = 0;
                }
                break;
            case ',':
                if (_line_state == 1) {
                    _term_offset = 0;
                    _term_number++;
                    fill_number();
                }
                _term_offset = 0;
                break;
            case '\r':
            case '\n':
                if (_line_state == 1) {
                    _term_offset = 0;
                    _term_number++;
                    fill_number();
                }
                if (_term_number >=5) {
                    handle_info();
                }
                _line_state = 0;
                _term_offset = 0;
                _term_number = 0;
                break;
            default:
                // ordinary characters
                if (_term_offset < sizeof(_term) - 1) {
                    _term[_term_offset] = temp;
                    _term_offset++;
                }
                break;
        }

    }


    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

bool UCam_Port_ASCII::detect_sentence() {
    if (strcmp(_term, "result0") == 0) {
        return true;
    }
    return false;
}

float UCam_Port_ASCII::get_number() {
    char *endptr = nullptr;
    float x = strtod(_term, &endptr);
    return x;
}

void UCam_Port_ASCII::fill_number()
{
    switch (_term_number) {
        case 1: // label
            break;
        case 2: // left
            _left = get_number();
            break;
        case 3: // right
            _right = get_number();
            break;
        case 4: // top
            _top = get_number();
            break;
        case 5: // bottom
            _bottom = get_number();
            break;
        default:
            break;
    }
}

void UCam_Port_ASCII::handle_info() {
    _frotend.display_info_p1 = _left;
    _frotend.display_info_p2 = _right;
    _frotend.display_info_p3 = _top;
    _frotend.display_info_p4 = _bottom;
    _frotend.display_info_new = true;
    _frotend.display_info_count++;
    bool _valid = false;
    _frotend._cam_state = 5;
    _valid = true;

    if (!_valid) {
        _frotend._n_count = 0;
        return;
    }

    float dt = (float)(millis() - _frotend._last_update_ms)*1.0e-3f;
    if (dt > 1.0f) {dt = 1.0f;}
    if (dt < 0.01f) {dt = 0.01f;} // sainty check, cam info will be at around 20Hz, conrresponding to 0.05s.
    
    float p1 = (_left + _right) * 0.5f;
    float p2 = (_top + _bottom) * 0.5f;

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
    _frotend.udpate_value(dt);
}

void UCam_Port_ASCII::do_cmd(float p1, float p2, float p3, float p4) {
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
