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

UCam_Port_UE4::UCam_Port_UE4(UCam &frotend_in):
    UCam_Port(frotend_in)
{
    ;
}

void UCam_Port_UE4::port_read() {
    static uint32_t last_update_ms = millis();
    uint32_t tnow = millis();

    FD2_msg2apm_ue4_gimbal &tmp_msg = copter.FD2_uart_msg_ue4.get_msg2apm_ue4_gimbal();
    if (tmp_msg._msg_1.updated) {
        float target_x = tmp_msg._msg_1.content.msg.x;
        float target_y = tmp_msg._msg_1.content.msg.y;
        handle_info(target_x, target_y);
        tmp_msg._msg_1.updated = false;   
    }

    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }
}

void UCam_Port_UE4::handle_info(float x, float y) {
    _frotend.display_info_p1 = x;
    _frotend.display_info_p2 = y;
    _frotend.display_info_p3 = 0;
    _frotend.display_info_p4 = 0;
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

    float p1 = x - copter.g2.user_parameters.cam_pixel_x.get() * 0.5f;
    float p2 = y - copter.g2.user_parameters.cam_pixel_y.get() * 0.5f;
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
    _frotend.correct_info.y = _frotend._cam_filter.get().z;
    _frotend._last_update_ms = millis();
    if (!_frotend._active) {
        _frotend._n_count += 1;
    }
    _frotend.update_value(dt);
}

void UCam_Port_UE4::do_cmd(float p1, float p2, float p3, float p4) {
    return;
}