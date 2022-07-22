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

UCapture::UCapture()
{
    ;
}

// initialise
void UCapture::init()
{
    _active = false;
    _cam_state = 0;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    _last_update_ms = 0;
    _target_yaw_rate_cds = 0.0f;
    _target_climb_rate = 0.0f;
    _n_count = 0;
    current_pos.zero();
    target_pos.zero();
    _q_angle_cd = 0.0f;
    _q_cds = 0.0f;
}


// clear return path and set home location.  This should be called as part of the arming procedure
void UCapture::handle_info(float p1, float p2, float p3, float p4)
{

    copter.Utarget.display_info_p1 = p1;
    copter.Utarget.display_info_p2 = p2;
    copter.Utarget.display_info_p3 = p3;
    copter.Utarget.display_info_p4 = p4;
    copter.Utarget.display_info_new = true;
    copter.Utarget.display_info_count++;


    correct_info.x = p1;
    correct_info.y = p2;

    target_pos.x = p1;
    target_pos.y = p2;
    target_pos.z = p3;



    _last_update_ms = millis();
    if (!_active) {
        _n_count += 1;
    }
}

int16_t UCapture::cam_state() {
    if (_active) {
        return 10;
    }
    return _cam_state;
}

const Vector2f& UCapture::get_raw_info() {
    return raw_info;
}

const Vector2f& UCapture::get_correct_info() {
    return correct_info;
}


// update
void UCapture::update()
{
    if (copter.g2.user_parameters.attack_pos_test.get()==1) {
        if (copter.position_ok()){
            current_pos.x = copter.inertial_nav.get_position().x;
            current_pos.y = copter.inertial_nav.get_position().y;
            current_pos.z = copter.inertial_nav.get_position().z;
        }
    }

    raw_info.x = current_pos.x;
    raw_info.y = current_pos.y;
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
    }

    if (!_active) {
        return;
    }

    // update q angle in cds
    static uint32_t _last_cal_ms = 0;
    _q_cds = 0.0f;
    float delta_t = float(millis() - _last_cal_ms)*0.001f; // in second
    float q_angle_cd = degrees(atan2f(target_pos.y - current_pos.y, target_pos.x - current_pos.x))*100.0f;
    float last_q = _q_angle_cd;
    _q_angle_cd = q_angle_cd;
    float new_q = _q_angle_cd;
    if (delta_t < 0.2f) {
        _q_cds = (new_q - last_q)/delta_t;
    }
    _last_cal_ms = millis();

    if (copter.g2.user_parameters.fly_yaw_tc.get() < 0.1f) {copter.g2.user_parameters.fly_yaw_tc.set_and_save(0.1f);}
    update_target_yaw_rate();
    // update_target_climb_rate();
}

void UCapture::update_target_yaw_rate() {
    float delta_yaw = wrap_180_cd(_q_angle_cd - copter.attitude_control->get_att_target_euler_cd().z);
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = delta_yaw / yaw_rate_tc;
    _target_yaw_rate_cds = constrain_float(yaw_rate_cds, -9000.0f, 9000.0f);
}

