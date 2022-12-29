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
    : _cam_filter(20.0f),
    _q_cds_filter(10.0f)
{
    ;
}

// initialise
void UCam::init()
{
    _active = false;
    _cam_state = 0;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    display_info_new = false;
    display_info_p1 = 0.0f;
    display_info_p2 = 0.0f;
    display_info_p3 = 0.0f;
    display_info_p4 = 0.0f;
    display_info_count = 0;
    _last_update_ms = 0;
    _target_pitch_rate = 0.0f;
    _target_yaw_rate_cds = 0.0f;
    _target_roll_angle = 0.0f;
    _current_angle_deg = 0.0f;
    _n_count = 0;
    _new_data = false;
    _port = NULL;
    init_port();
}


// clear return path and set home location.  This should be called as part of the arming procedure
void UCam::handle_info(const mavlink_command_long_t* packet)
{
    float p1 = packet->param1;
    float p2 = packet->param2;
    float p3 = packet->param3;
    float p4 = packet->param4;
    float p5 = packet->param5;
    float p6 = packet->param6;
    float p7 = packet->param7;

    display_info_p1 = p1;
    display_info_p2 = p2;
    display_info_p3 = p3;
    display_info_p4 = p4;
    display_info_new = true;
    display_info_count++;
    bool _valid = false;
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 0 && (int16_t)p5 == 0) {
    // self check fail (2Hz)
        _cam_state = 1;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 1) {
    // self check pass (2Hz)
        _cam_state = 2;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 3) {
    // handle cmd from apm
        _cam_state = 3;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 2) {
    // working
        _cam_state = 4;
    }
    if ((int16_t)p3 == 1 && (int16_t)p4 == 1) {
    // target following
        _cam_state = 5;
        _valid = true;
    }
    if ((int16_t)p1 == 0 && (int16_t)p2 == 0 && (int16_t)p3 == 0 && (int16_t)p4 == 0 && (int16_t)p5 == 13) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "Para [%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f]", p1, p2, p3, p4, p5, p6, p7);
        mavlink_command_long_t new_packet;
        new_packet.param1 = 11.0f;
        new_packet.param2 = 0.0f;
        new_packet.param3 = p6;
        new_packet.param4 = p7;
        new_packet.param5 = 0.0f;
        new_packet.param6 = 0.0f;
        new_packet.param7 = 0.0f;
        copter.send_my_command_long((mavlink_channel_t)0, &new_packet);
    }

    if (!_valid) {
        _n_count = 0;
        return;
    }

    float dt = (float)(millis() - _last_update_ms)*1.0e-3f;
    if (dt > 1.0f) {dt = 1.0f;}
    if (dt < 0.01f) {dt = 0.01f;} // sainty check, cam info will be at around 20Hz, conrresponding to 0.05s.
    
    raw_info.x = p1;
    raw_info.y = p2;
    Matrix3f tmp_m;
    if (is_zero(copter.g2.user_parameters.fly_roll_factor)) {
        tmp_m.from_euler(0.0f, 0.0f, 0.0f);
    } else {
        tmp_m.from_euler(copter.ahrs_view->roll, 0.0f, 0.0f);
    }
    Vector3f tmp_input = Vector3f(100.f,p1,-p2);
    Vector3f tmp_output = tmp_m*tmp_input;
    _cam_filter.apply(tmp_output, dt);
    correct_info.x = _cam_filter.get().y;
    correct_info.y = -_cam_filter.get().z;
    _last_update_ms = millis();
    if (!_active) {
        _n_count += 1;
    }
    udpate_value(dt);
}

void UCam::udpate_value(float dt){
    if (copter.g2.user_parameters.cam_pixel_x.get() < 50.f) {copter.g2.user_parameters.cam_pixel_x.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_pixel_y.get() < 50.f) {copter.g2.user_parameters.cam_pixel_y.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_angle_x.get() < 30.f) {copter.g2.user_parameters.cam_angle_x.set_and_save(30.f);}
    if (copter.g2.user_parameters.cam_angle_y.get() < 30.f) {copter.g2.user_parameters.cam_angle_y.set_and_save(30.f);}
    if (copter.g2.user_parameters.fly_yaw_tc.get() < 0.1f) {copter.g2.user_parameters.fly_yaw_tc.set_and_save(0.1f);}
    update_target_pitch_rate();
    update_target_roll_angle();
    update_target_yaw_rate();
    update_target_track_angle();
    update_q_rate_cds(dt);
}

int16_t UCam::cam_state() {
    if (_active) {
        return 10;
    }
    return _cam_state;
}

const Vector2f& UCam::get_raw_info() {
    return raw_info;
}

const Vector2f& UCam::get_correct_info() {
    return correct_info;
}


void UCam::init_port()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CamMavlink, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "CAM MAV init");
        _Ucam_port = new UCam_Port_Mavlink(*this);
        return;
    }

    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CamASCII, 0);
    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "CAM ASCII init");
        _Ucam_port = new UCam_Port_ASCII(*this);
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "CAM MAV init FAILED");
}

void UCam::port_read()
{
    if (get_port() == NULL) {return;}
    if (_Ucam_port == nullptr) {return;}
    _Ucam_port->port_read();
}


void UCam::do_cmd(float p1, float p2, float p3, float p4)
{
    if (get_port() == NULL) {return;}
    if (_Ucam_port == nullptr) {return;}
    _Ucam_port->do_cmd(p1, p2, p3, p4);
}


// update
void UCam::update()
{
    port_read();
    time_out_check();
}

void UCam::time_out_check() {
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _n_count = 0;
        _active = false;
        _cam_state = 0;
        raw_info.x = 0.0f;
        raw_info.y = 0.0f;
    } else if (_n_count > 10) {
        _active = true;
    } else {
        _active = false;
    }

    if (!_active) {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate_cds = 0.0f;
        _current_angle_deg = copter.g2.user_parameters.fly_attack_angle*0.01f;
        _q_rate_cds = 0.0f;
        _q_cds_filter.reset(0.0f);
        return;
    }
}

void UCam::update_target_pitch_rate() {
    float measurement = (get_correct_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y); //-1 to +0.1
    float my_target_pitch_rate = -1.0f*copter.g2.user_parameters.Ucam_pid.update_all(copter.g2.user_parameters.cam_target_y, measurement, false)*copter.g2.user_parameters.fly_pitch_rate.get();
    if (my_target_pitch_rate > 0.0f) {
        my_target_pitch_rate *= 1.0f;
    }
    _target_pitch_rate = my_target_pitch_rate ;
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate);
}

void UCam::update_target_roll_angle() {
    // float info_x = copter.Ucam.get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    // info_x = constrain_float(info_x*copter.g2.user_parameters.fly_roll_factor, -1.0f, 1.0f);
    // float pitch_limit = MAX(copter.g2.user_parameters.fly_roll_limit, degrees(fabsf(copter.ahrs_view->pitch)));
    // _target_roll_angle = pitch_limit*info_x;

    float info_x = copter.Ucam.get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    float out_x = copter.g2.user_parameters.Roll_pid.update_all(0.0f, -info_x, false);
    float pitch_limit = MAX(copter.g2.user_parameters.fly_roll_limit, degrees(fabsf(copter.ahrs_view->pitch)));
    _target_roll_angle = pitch_limit*out_x;
}

void UCam::update_target_yaw_rate() {
    float info_x = copter.Ucam.get_correct_info().x/(0.5f*copter.g2.user_parameters.cam_pixel_x) - copter.g2.user_parameters.cam_target_x.get();
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = constrain_float((3000.f * info_x * yaw_rate_tc), -3000.f, 3000.f);
    _target_yaw_rate_cds = yaw_rate_cds;
}

void UCam::update_target_track_angle() {
    _current_angle_deg = -degrees(copter.userhook_FastLoop_pitch_get()) - (copter.g2.user_parameters.cam_pitch_offset)*0.01f-degrees(atanf((copter.Ucam.get_correct_info().y)/(0.5f*copter.g2.user_parameters.cam_pixel_y)*tanf(0.5f*radians(copter.g2.user_parameters.cam_angle_y))));
}

void UCam::update_q_rate_cds(float dt) {
    static float last_q_cd = 0.0f;
    float q_cd = 100.f*degrees(copter.ahrs_view->yaw) + degrees(atanf((copter.Ucam.get_correct_info().x)/(0.5f*copter.g2.user_parameters.cam_pixel_x)*tanf(0.5f*radians(copter.g2.user_parameters.cam_angle_x))));
    float q_cds = (q_cd - last_q_cd)/dt;
    last_q_cd = q_cd;
    _q_cds_filter.apply(q_cds, dt);
    _q_rate_cds = _q_cds_filter.get();
}
