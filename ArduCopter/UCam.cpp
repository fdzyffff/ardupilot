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
//#include "UCam.h"

UCam::UCam()
{    init();
}

// initialise
void UCam::init()
{
    _active = false;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    _last_update_ms = 0;
    _target_pitch_rate = 0.0f;
    _target_yaw_rate_cds = 0.0f;
    _target_roll_angle = 0.0f;
}

// clear return path and set home location.  This should be called as part of the arming procedure
void UCam::handle_info(float input_x, float input_y, bool valid)
{
    if (!valid) {return;}
    raw_info.x = input_x;
    raw_info.y = input_y;
    Matrix3f tmp_m;
    if (is_zero(copter.g2.user_parameters.fly_roll_factor)) {
        tmp_m.from_euler(0.0f, 0.0f, 0.0f);
    } else {
        tmp_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    }
    Vector3f tmp_input = Vector3f(100.f,input_x,-input_y);
    Vector3f tmp_output = tmp_m*tmp_input;
    correct_info.x = tmp_output.y;
    correct_info.y = -tmp_output.z;
    _last_update_ms = millis();
}

// update
void UCam::update()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _active = false;
    } else {
        _active = true;
    }

    if (!_active) {
        return;
    }
    update_target_pitch_rate();
    update_target_roll_angle();
    update_target_yaw_rate();
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
    float info_x = copter.Ucam.get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    _target_roll_angle = copter.g2.user_parameters.fly_roll_limit*info_x*copter.g2.user_parameters.fly_roll_factor ;
}

void UCam::update_target_yaw_rate() {
    float info_x = copter.Ucam.get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    float x_angle = copter.g2.user_parameters.cam_angle_x;
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = 100.f * (x_angle * info_x / yaw_rate_tc);
    _target_yaw_rate_cds = yaw_rate_cds;
}