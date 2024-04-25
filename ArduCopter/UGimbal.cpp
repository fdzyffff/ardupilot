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
    ;
}

// initialise
void UGimbal::init()
{
    _target_vel.zero();
    _target_pos.zero();
    _current_pos.zero();
    _current_pos.z = 1000.f;
    _target_yaw_cd = 0.0f;
    _target_dir_rate_cd = 0.0f;
    _last_ms = 0;
    _active = false;
    _new_data = false;
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

void UGimbal::handle_info(float pitch_in, float roll_in, float yaw_in, float abs_yaw_in)
{
    display_info.p1 = pitch_in;
    display_info.p2 = roll_in;
    display_info.p3 = yaw_in;
    display_info.p4 = abs_yaw_in;
    display_info.count++;

    _dir_rate_filter.update(yaw_in, millis());

    float target_dir_rate = _dir_rate_filter.slope()*1000.f;

    _target_yaw_cd = wrap_360(degrees(copter.ahrs_view->yaw) + yaw_in) * 100.f;
    // float delta_yaw = wrap_360(yaw_in);

    // float vel_max = copter.wp_nav->get_default_speed_xy();
    float safe_pitch_in = constrain_float(pitch_in, -80.f, 80.f);

    float target_vel = 100.f*copter.g2.user_parameters.vel_p.get();
    // float xy_comp = cosf(radians(safe_pitch_in));

    _target_vel.x = target_vel;
    _target_vel.y = 0.0f;
    _target_vel.z = tanf(radians(safe_pitch_in))*target_vel;

    // gcs().send_text(MAV_SEVERITY_WARNING, "%f, %f, %f",_target_vel.x, tanf(radians(safe_pitch_in)), safe_pitch_in);

    _target_dir_rate_cd = copter.g2.user_parameters.vel_kp.get()*target_dir_rate*100.f;

    _new_data = true;

    display_info.p11 = _target_vel.x;
    display_info.p12 = _target_vel.y;
    display_info.p13 = _target_vel.z;
    display_info.p14 = _target_dir_rate_cd;
}

void UGimbal::handle_info(int16_t tracking_status)
{
    //gcs().send_text(MAV_SEVERITY_WARNING, "Target state: %d",tracking_status);
    if (tracking_status != 0) {
        _last_ms = millis();
    }
}

// update 
void UGimbal::update()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if (_time_out != 0 && ( ((now - _last_ms) > _time_out)||(_last_ms == 0) ) )  {
        if (_active) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target lost");
        }
        _active = false;
        //gcs().send_text(MAV_SEVERITY_WARNING, "----_target_vel.zero()----");
        _target_vel.zero();
    } else {
        if (!_active) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target aquire");
            //copter.set_mode(Mode::Number::GIMBALFOLLOW, ModeReason::MISSION_END);
        }
        _active = true;
    }

}
