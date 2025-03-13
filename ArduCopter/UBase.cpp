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

UBase::UBase()
{
    ;
}

// initialise
void UBase::init()
{
    _last_ms = 0;
    _valid = false;
    // _filter_target_cm.set_cutoff_frequency(30.0f, 20.f);
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p21 = 0.0f;
    display_info.p22 = 0.0f;
    display_info.p23 = 0.0f;
    display_info.count = 0;
    display_info.count_log = 0;
    display_info.new_data = false;
    _target_pitch = 0.0f;
    _target_roll = 0.0f;
    _target_yaw = 0.0f;
    _base_roll = 0.0f;
    _base_pitch = 0.0f;
    _base_yaw = 0.0f;
    // _mode = 0;
    _initialized = true;

    // gcs().send_text(MAV_SEVERITY_INFO, "FD1_uart_K230.init()");
}

void UBase::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialized) {return;}
    // if (_target_sys_id != 0 && _target_sys_id != msg.sysid) {return;}

    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        _valid = true;
        _last_ms = millis();
        display_info.count++;

        // decode packet
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        
        _base_roll = attitude.roll;
        _base_pitch = attitude.pitch;
        // _base_yaw = attitude.yaw;

        // Vector3f tmp_in = Vector3f(attitude.roll, attitude.pitch, 0.0f);
        // Matrix3f tmp_bf_m;
        // tmp_bf_m.from_euler(0.0f, 0.0f, attitude.yaw);
        // Vector3f tmp_out = tmp_bf_m*tmp_in;
        // _base_roll = tmp_out.x;
        // _base_pitch = tmp_out.y;


        display_info.p1 = degrees(attitude.pitch)*100.f;
        display_info.p2 = degrees(attitude.yaw)*100.f;
        display_info.p3 = degrees(_base_roll)*100.f;
        display_info.p4 = degrees(_base_pitch)*100.f;

    }
}

// update 
void UBase::update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
    }

    update_valid();
    update_target_angle();

    display_info.p3 = _target_roll;
    display_info.p4 = _target_pitch;
}

void UBase::update_valid()
{
    const uint32_t now = millis();
    uint32_t _time_out = 2000;
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Base lost");
        }
        _valid = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        // _raw_target_cm.zero();
        // _filter_target_cm.reset();

        _base_roll = 0.0f;
        _base_pitch = 0.0f;
        _base_yaw = 0.0f;
    } else {
        if (!_valid) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Base aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _valid = true;
    }
}

// degree/second
void UBase::update_target_angle()
{
    if (copter.g2.user_parameters.angle_mode.get() == 0) {
        RC_Channel *roll_4x4_ch = rc().find_channel_for_option(RC_Channel::aux_func_t::ROLL_4X4);
        RC_Channel *pitch_4x4_ch = rc().find_channel_for_option(RC_Channel::aux_func_t::PITCH_4X4);
        if ((roll_4x4_ch != nullptr) && (roll_4x4_ch->get_radio_in() > 0)) {
            _target_roll = roll_4x4_ch->norm_input_dz()*4500.f;
        }
        if ((pitch_4x4_ch != nullptr) && (pitch_4x4_ch->get_radio_in() > 0)) {
            _target_pitch = pitch_4x4_ch->norm_input_dz()*4500.f;
        }
    } else {
        _target_roll = degrees(_base_roll)*100.f;
        _target_pitch = degrees(_base_pitch)*100.f;
    }
    _target_roll = constrain_float(_target_roll, -1500.0f, 1500.f);
    _target_pitch = constrain_float(_target_pitch, -1500.0f, 1500.f);
}

void UBase::set_mode(uint8_t mode_in)
{
    copter.g2.user_parameters.angle_mode.set(mode_in);
    if (copter.g2.user_parameters.angle_mode.get() == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "Manual FOLLOW");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Auto FOLLOW");
    }
}
