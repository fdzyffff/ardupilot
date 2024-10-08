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

UK230::UK230()
{
    FD1_uart_K230.init();
    FD1_uart_K230.get_msg_K230().set_enable();
    _raw_target_cm.zero();
}

// initialise
void UK230::init()
{
    _last_ms = 0;
    _valid = false;
    _new_data = false;
    _filter_target_cm.set_cutoff_frequency(30.0f, 20.f);
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.count = 0;
}

void UK230::read_uart()
{
    FD1_uart_K230.read();
    FD1_msg_K230 &tmp_msg = FD1_uart_K230.get_msg_K230();
    if (tmp_msg._msg_1.updated) {
        display_info.count++;
        if (tmp_msg._msg_1.content.msg.t1_ok) {
            _new_data = true;
            _last_ms = millis();
            display_info.p1 = tmp_msg._msg_1.content.msg.t1_x;
            display_info.p2 = tmp_msg._msg_1.content.msg.t1_y;
            display_info.p3 = tmp_msg._msg_1.content.msg.t1_z;
            display_info.p4 = 1;
            get_target(1, tmp_msg._msg_1.content.msg.t1_x, tmp_msg._msg_1.content.msg.t1_y, tmp_msg._msg_1.content.msg.t1_z);
        }
        else if (tmp_msg._msg_1.content.msg.t2_ok) {
            _new_data = true;
            _last_ms = millis();
            display_info.p1 = tmp_msg._msg_1.content.msg.t2_x;
            display_info.p2 = tmp_msg._msg_1.content.msg.t2_y;
            display_info.p3 = tmp_msg._msg_1.content.msg.t2_z;
            display_info.p4 = 2;
            get_target(2, tmp_msg._msg_1.content.msg.t1_x, tmp_msg._msg_1.content.msg.t1_y, tmp_msg._msg_1.content.msg.t1_z);
        } else {
            display_info.p1 = tmp_msg._msg_1.content.msg.t2_x;
            display_info.p2 = tmp_msg._msg_1.content.msg.t2_y;
            display_info.p3 = tmp_msg._msg_1.content.msg.t2_z;
            display_info.p4 = 0;
        }

        tmp_msg._msg_1.updated = false;   
    }
}

void UK230::get_target(int8_t tag_num, float x_in, float y_in, float z_in)
{
    _raw_target_cm.x = x_in;
    _raw_target_cm.y = y_in;
    _raw_target_cm.z = z_in;

    display_info.p11 = _filter_target_cm.get().x;
    display_info.p12 = _filter_target_cm.get().y;
    display_info.p13 = _filter_target_cm.get().z;
    display_info.p14 = 0;
}

// update 
void UK230::update()
{
    read_uart();
    update_valid();
}

void UK230::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target lost");
        }
        _valid = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        _raw_target_cm.zero();
        _filter_target_cm.reset();
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }

}
