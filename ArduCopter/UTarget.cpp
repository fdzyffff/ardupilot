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

UTarget::UTarget()
{
    ;
}

// initialise
void UTarget::init()
{
    _type = copter.g2.user_parameters.target_type.get();

    display_info_new = false;
    display_info_p1 = 0.0f;
    display_info_p2 = 0.0f;
    display_info_p3 = 0.0f;
    display_info_p4 = 0.0f;
    display_info_count = 0;

    if (_type == 0) {
        Ucam.init();
    }
}


// clear return path and set home location.  This should be called as part of the arming procedure
void UTarget::handle_info(float p1, float p2, float p3, float p4)
{
    if (_type == 0) {
        Ucam.handle_info(p1,p2,p3,p4);
    }
}

int16_t UTarget::cam_state() {
    if (_type == 0) {
        return Ucam.cam_state();
    }
    return 0;
}

bool UTarget::is_active() {
    if (_type == 0) {
        return Ucam.is_active();
    }
    if (_type == 1) {
        return Ucapture.is_active();
    }
    return false;
}

const Vector2f& UTarget::get_raw_info() {
    if (_type == 0) {
        return Ucam.get_raw_info();
    }
    return Ucapture.get_raw_info();
}

const Vector2f& UTarget::get_correct_info() {
    if (_type == 0) {
        return Ucam.get_correct_info();
    }
    return Ucapture.get_correct_info();
}


void UTarget::do_cmd(float p1)
{
    if (_type == 0) {
        Ucam.do_cmd(p1);
    }
}


// update
void UTarget::update()
{
    if (_type == 0) {
        Ucam.update();
    }
    if (_type == 1) {
        Ucapture.update();
    }
}

float UTarget::get_target_pitch_rate() {
    if (_type == 0) {
        return Ucam.get_target_pitch_rate();
    }
    return 0.0f;
}
float UTarget::get_target_roll_angle() {
    if (_type == 0) {
        return Ucam.get_target_roll_angle();
    }
    return 0.0f;
}
float UTarget::get_target_yaw_rate() {
    if (_type == 0) {
        return Ucam.get_target_yaw_rate();
    }
    if (_type == 1) {
        return Ucapture.get_target_yaw_rate();
    }
    return 0.0f;
}
float UTarget::get_current_angle_deg() {
    if (_type == 0) {
        return Ucam.get_current_angle_deg();
    }
    return 0.0f;
}