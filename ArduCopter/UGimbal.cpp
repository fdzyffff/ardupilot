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

UGimbal::UGimbal()
{
    uart_msg_gimbal2gcs.set_enable();//没用，主要是仪式感
}

// initialise
void UGimbal::init()
{
    _target_vel.zero();
    _target_pos.zero();
    _current_pos.zero();
    _current_pos.z = 1000.f;
    _target_yaw_cd = 0.0f;
    _last_ms = 0;
    _valid = false;
    _new_data = false;
    _filter_yaw_in.set_cutoff_frequency(20.0f, 1.f);
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

void UGimbal::handle_info(float pitch_in, float roll_in, float yaw_in)
{
    display_info.p1 = pitch_in;
    display_info.p2 = roll_in;
    display_info.p3 = yaw_in;
    display_info.p4 = 0;
    display_info.count++;

    float pitch_out = pitch_in;
    float roll_out = roll_in;
    float t_yaw_out = wrap_360(degrees(copter.ahrs_view->yaw) + yaw_in);
    _filter_yaw_in.apply(t_yaw_out);
    float yaw_out = _filter_yaw_in.get();

    float vel_max = copter.wp_nav->get_default_speed_xy();
    float pitch_safe = constrain_float(fabsf(pitch_out), 10.f, 100.f);
    float target_angle = constrain_float(copter.g2.user_parameters.cam_angle.get(), 10.f, 90.f);
    float current_dist_unit = cosf(radians(pitch_safe)) / sinf(radians(pitch_safe));//cot(pitch)
    float target_dist_unit = cosf(radians(target_angle)) / sinf(radians(target_angle));//cot(pitch)
    float error_dist_unit = target_dist_unit-current_dist_unit;
    error_dist_unit = constrain_float(error_dist_unit, -2.0f, 2.0f);//avoid extreme value

    float target_vel_unit = copter.g2.user_parameters.Ucam_pid.update_all(0.0f, error_dist_unit, false);

    _target_vel.x = vel_max*constrain_float(target_vel_unit, -1.0f, 1.0f);

    _target_yaw_cd = yaw_out*100.f;

    _new_data = true;

    display_info.p11 = pitch_out;
    display_info.p12 = roll_out;
    display_info.p13 = yaw_out;
    display_info.p14 = _target_vel.x;

    // Matrix3f ned_to_gimbal;
    // ned_to_gimbal.from_euler(0.0f, 0.0f, radians(yaw_out));
}

float UGimbal::get_target_dist()
{
    return _target_pos.length();
}

void UGimbal::read_byte(uint8_t temp)
{
    uart_msg_gimbal2gcs.parse(temp);
    if (uart_msg_gimbal2gcs._msg_1.updated) {
        uart_msg_gimbal2gcs._msg_1.updated = false;
        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        _last_ms = millis();
        handle_info(pitch, roll, yaw);
    }
}

void pack_msg()
{
    if (uart_msg_gcs2gimbal._msg_1.updated) {
        uart_msg_gcs2gimbal._msg_1.updated = false;
        // copy to apm2gimbal
    }
    // add apm info to apm2gimbal
    uart_msg_apm2gimbal.make_sum();
    uart_msg_apm2gimbal._msg_1.need_send = true;
}

// update 
void UGimbal::update()
{
    update_valid();
    pack_msg();
}

void UGimbal::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target lost");
        }
        _valid = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        _target_vel.zero();
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }

}
