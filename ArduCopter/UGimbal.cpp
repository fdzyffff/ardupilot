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

    _target_pitch_cd = 0.0f;
    _target_roll_cd = 0.0f;
    _target_yaw_cd = 0.0f;
    _target_climb_rate = 0.0f;//cm/s
    _last_ms = 0;
    _valid = false;
    _new_data = false;
    _filter_yaw_in.set_cutoff_frequency(30.0f, 20.f);
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

void UGimbal::handle_info(float pitch_in, float roll_in, float yaw_in) // degree
{
    _last_ms = millis();
    display_info.p1 = pitch_in;
    display_info.p2 = roll_in;
    display_info.p3 = yaw_in;
    display_info.p4 = 0;
    display_info.count++;

    float yaw_offset = copter.g2.user_parameters.attack_yaw_offset.get();
    float pitch_offset = copter.g2.user_parameters.attack_pitch_offset.get();
    pitch_in = constrain_float(pitch_in-pitch_offset, -80.0f, 80.0f);

    float tan_pitch = sinf(radians(pitch_in)) / cosf(radians(pitch_in));
    float delta_z_m = 2.0f * tan_pitch;
    _target_climb_rate = copter.g2.user_parameters.Ucam_pid.update_all(0.0f, -delta_z_m, false)*100.f;

    // gcs().send_text(MAV_SEVERITY_WARNING, "%f, %f, %f", tan_pitch, delta_z_m, _target_climb_rate);

    float t_yaw_out = wrap_360(degrees(copter.ahrs_view->yaw) + yaw_in - yaw_offset);
    _filter_yaw_in.apply(t_yaw_out);
    float yaw_out = _filter_yaw_in.get();
    _target_yaw_cd = yaw_out*100.f;

    _target_pitch_cd = pitch_in*100.f;
    _target_roll_cd = constrain_float(yaw_in * copter.g2.user_parameters.attack_roll_factor.get(), -15.f, 15.f)*100.f;

    _new_data = true;

    display_info.p11 = _target_pitch_cd;
    display_info.p12 = _target_roll_cd;
    display_info.p13 = _target_yaw_cd;
    display_info.p14 = _target_climb_rate;
}

void UGimbal::read_status_byte(uint8_t temp)
{
    uart_msg_gimbal2gcs.parse(temp);
    if (uart_msg_gimbal2gcs._msg_1.updated) {
        uart_msg_gimbal2gcs._msg_1.updated = false;
        float pitch = uart_msg_gimbal2gcs._msg_1.content.msg.angle_pitch/100;
        float roll = uart_msg_gimbal2gcs._msg_1.content.msg.angle_roll/100;
        float yaw = uart_msg_gimbal2gcs._msg_1.content.msg.angle_yaw/100;
        handle_info(pitch, roll, yaw);
    }
}

void UGimbal::read_command_byte(uint8_t temp)
{
    uart_msg_gcs2gimbal.parse(temp);
    pack_msg();
}

void UGimbal::pack_msg()
{
    if (uart_msg_gcs2gimbal._msg_1.updated) {
        uart_msg_gcs2gimbal._msg_1.updated = false;
        // copy to apm2gimbal
        memcpy(uart_msg_apm2gimbal._msg_1.content.data, 
            uart_msg_gcs2gimbal._msg_1.content.data, 
            uart_msg_gcs2gimbal._msg_1.length*sizeof(uint8_t));
    }
    // add apm info to apm2gimbal
    uart_msg_gcs2gimbal._msg_1.content.msg.msg_apm.angle_pitch = wrap_180_cd(copter.ahrs.pitch_sensor);
    uart_msg_gcs2gimbal._msg_1.content.msg.msg_apm.angle_roll = wrap_180_cd(copter.ahrs.roll_sensor);
    uart_msg_gcs2gimbal._msg_1.content.msg.msg_apm.angle_yaw = wrap_360_cd(copter.ahrs.yaw_sensor);
    // make sum and set flag to send
    uart_msg_apm2gimbal.make_sum();
    uart_msg_apm2gimbal._msg_1.need_send = true;
}

// update 
void UGimbal::update()
{
    update_valid();
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
        _target_pitch_cd = 0.0f;
        _target_roll_cd = 0.0f;
        _target_yaw_cd = 0.0f;
        _target_climb_rate = 0.0f;//cm/s
        copter.g2.user_parameters.Ucam_pid.reset_I();
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }

}
