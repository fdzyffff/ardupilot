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

UGround::UGround()
{
    init();
}

// initialise
void UGround::init()
{
    _active = false;
    _last_update_ms = 0;
    _last_state_update_ms = 0;
    _state_timer_ms = 0;
    _position_id = 0;
    _group_id = 1;
    _is_leader = true;
    _distance = 200.f;
    _cmd = 0;
    set_state(UGCS_None);
    _dest_loc_vec.zero();
    _dest_vel_vec.zero();
    _dest_yaw_cd = 0.0f;
}

void UGround::handle_info(int16_t cmd, int16_t group_id, int16_t distance, int16_t free)
{
    group_id = 1;
    _group_id = group_id;
    _distance = distance;
    _cmd = cmd;
    do_cmd(_cmd, false);
    my_group1.set_distance(_distance);
    my_group2.set_distance(_distance);
}

void UGround::do_cmd(int16_t cmd, bool force_set) {
    switch (cmd) {
        case 2:
            set_state(UGCS_Takeoff);
            break;
        case 3:
            set_state(UGCS_Curise, force_set);
            break;
        case 4:
            if (!is_leader()) {
                set_state(UGCS_Assemble, force_set);
            }
            break;
        case 5:
            set_state(UGCS_Attack);
            break;
        case 6:
            set_state(UGCS_FS1);
            break;
        default:
            break;
    }
}

void UGround::refresh_cmd() {
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "(%d)cmd:%d",_is_leader,_cmd);
    do_cmd(_cmd, true);
}

void UGround::set_up_offset(int8_t sender_id, Vector3f target_postion, Vector3f target_velocity, float target_heading) {
    Vector3f offset_position = Vector3f(0.0f, 0.0f, 0.0f);
    switch (_group_id) {
        case 1:
            offset_position = my_group1.get_offset(copter.g.sysid_this_mav, sender_id);
            break;
        case 2:
            offset_position = my_group2.get_offset(copter.g.sysid_this_mav, sender_id);
            break;
        default:
            break;
    }

    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(copter.g2.user_parameters.gcs_group_yaw));
    offset_position = tmp_m*offset_position;

    _dest_loc_vec = target_postion + offset_position;
    _dest_vel_vec = target_velocity;
    _dest_yaw_cd = target_heading*100.f;
}

// update
void UGround::update()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.gcs_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _active = false;
    } else {
        _active = true;
    }
    state_update();
}

void UGround::state_update() 
{
    uint32_t timer = millis() - _last_state_update_ms;
    if (copter.control_mode == Mode::Number::STABILIZE) {
        set_state(UGCS_None);
    }

    float sonar_height = -10.0f;
    if (copter.rangefinder_alt_ok()) {sonar_height = copter.rangefinder_state.alt_cm;}
    switch (get_state()) {
        case UGCS_Takeoff:
        {
            if ( (copter.control_mode == Mode::Number::GUIDED) && (copter.mode_guided.mode() == Guided_WP) && timer>_state_timer_ms) {
                set_state(UGCS_Standby1);
                _state_timer_ms = 3000;
            }
            break;
        }
        case UGCS_Standby1:
        {
            if (timer>_state_timer_ms) {
                set_state(UGCS_Fly);
            }
            break;
        }
        case UGCS_Fly:
        {
            if (is_leader()){
                if ((copter.mode_auto.mission.get_current_nav_index() > copter.g2.user_parameters.gcs_num_cruise) ) {
                    set_state(UGCS_Standby2);
                    _state_timer_ms = 300000;
                }
            } else {
                copter.mode_guided.set_destination_posvel(_dest_loc_vec, _dest_vel_vec, true, _dest_yaw_cd, false, 0.0f, false);
            }
            break;
        }
        case UGCS_Standby2:
        {
            if (timer>_state_timer_ms) {
                set_state(UGCS_FS1);
            }
            break;
        }
        case UGCS_Curise:
        {
            if (copter.Ucam.is_active()) {
                set_state(UGCS_Lockon);
                _state_timer_ms = 300000;
            }
            if (!is_leader()) {
                copter.mode_guided.set_destination_posvel(_dest_loc_vec, _dest_vel_vec, true, _dest_yaw_cd, false, 0.0f, false);
            }
            break;
        }
        case UGCS_Assemble:
        {
            if (!is_leader()) {
                copter.mode_guided.set_destination(_dest_loc_vec, true, _dest_yaw_cd, false, 0.0f, false);
                if (copter.flightmode->wp_distance() < 100.f) {
                    set_state(UGCS_Lockon);
                    _state_timer_ms = 300000;
                }
            }
            break;
        }
        case UGCS_Lockon:
        {
            if (timer>_state_timer_ms) {
                set_state(UGCS_FS1);
            }
            break;
        }
        case UGCS_Attack:
        {
            if (sonar_height > 0 && sonar_height < 100 && !copter.Ucam.is_active()) {
                set_state(UGCS_Lockon);
            }
            break;
        }
        case UGCS_FS1:
        {
            break;
        }
        default:
            break;
    }
}

void UGround::set_state(UGCS_state_t new_state, bool force_set)
{
    if ( (get_state() == new_state) && !force_set ) {
        return;
    }
    bool ret = false;
    switch (new_state) {
        case UGCS_None:
            ret = true;
            break;
        case UGCS_Takeoff:
            ret = (copter.arming.arm(AP_Arming::Method::MAVLINK) && copter.Ugcs_do_takeoff());
            _state_timer_ms = 1000;
            break;
        case UGCS_Fly:
            ret = copter.Ugcs_do_fly();
            break;
        case UGCS_Standby1:
        case UGCS_Standby2:
            ret = copter.Ugcs_do_standby();
            break;
        case UGCS_Curise:
            ret = copter.Ugcs_do_cruise();
            break;
        case UGCS_Assemble:
            ret = copter.Ugcs_do_assemble();
            break;
        case UGCS_Lockon:
            ret = copter.Ugcs_do_lockon();
            break;
        case UGCS_Attack:
            ret = copter.Ugcs_do_attack();
            break;
        case UGCS_FS1:
            ret = copter.Ugcs_do_fs1();
            break;
        default:
            break;
    }
    if (!ret) {
        set_state(UGCS_FS1);  
        return;
    }
    _state = new_state;
    _last_state_update_ms = millis();
    switch (_state) {
        case UGCS_None:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_None");
            break;
        case UGCS_Takeoff:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Takeoff");
            break;
        case UGCS_Fly:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Fly");
            break;
        case UGCS_Standby1:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Standby1");
            break;
        case UGCS_Standby2:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Standby2");
            break;
        case UGCS_Curise:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Curise");
            break;
        case UGCS_Assemble:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Assemble");
            break;
        case UGCS_Lockon:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Lockon");
            break;
        case UGCS_Attack:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_Attack");
            break;
        case UGCS_FS1:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "UGCS_FS1");
            break;
        default:
            break;
    }
}

int16_t UGround::get_state_num() {
    int16_t ret = -1;
    if (is_leader()) {
        switch (_state) {
            case UGCS_None:
                ret = 0;
                break;
            case UGCS_Takeoff:
                ret = 1;
                break;
            case UGCS_Fly:
            case UGCS_Standby1:
                ret = 2;
                break;
            case UGCS_Standby2:
            case UGCS_Curise:
                ret = 4;
                break;
            case UGCS_Lockon:
            case UGCS_Assemble:
                ret = 5;
                break;
            case UGCS_Attack:
                ret = 6;
                break;
            case UGCS_FS1:
            default:
                break;
        }
    } else {
        switch (_state) {
            case UGCS_None:
                ret = 0;
                break;
            case UGCS_Takeoff:
                ret = 1;
                break;
            case UGCS_Fly:
            case UGCS_Standby1:
            case UGCS_Standby2:
            case UGCS_Curise:
                ret = 3;
                break;
            case UGCS_Assemble:
            case UGCS_Lockon:
                ret = 5;
                break;
            case UGCS_Attack:
                ret = 6;
                break;
            case UGCS_FS1:
            default:
                break;
        }
    }
    return ret;
}

bool Copter::Ugcs_do_takeoff() // takeoff
{
    if (set_mode(Mode::Number::GUIDED, ModeReason::TOY_MODE)) {
        mode_guided.do_user_takeoff(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f),true);
        return true;
    }
    return false;
}

bool Copter::Ugcs_do_fly()     // fly
{
    if (Ugcs.is_leader()) {
        if (set_mode(Mode::Number::AUTO, ModeReason::TOY_MODE)) {
            mode_auto.mission.set_current_cmd(1);
            return true;
        }
    } else {
        return set_mode(Mode::Number::GUIDED, ModeReason::TOY_MODE);
    }
    return false;
}

bool Copter::Ugcs_do_standby() // loiter
{
    bool ret = set_mode(Mode::Number::BRAKE, ModeReason::TOY_MODE);
    return ret;
}

bool Copter::Ugcs_do_cruise()  // fly and search
{
    if (Ugcs.is_leader()) {
        if (set_mode(Mode::Number::AUTO, ModeReason::TOY_MODE)) {
            mode_auto.mission.set_current_cmd(g2.user_parameters.gcs_num_cruise);
            return true;
        }
    } else {
        return set_mode(Mode::Number::GUIDED, ModeReason::TOY_MODE);
    }
    return false;
}

bool Copter::Ugcs_do_assemble()  // assemble
{
    if (Ugcs.is_leader()) {
        return set_mode(Mode::Number::LOCKON, ModeReason::TOY_MODE);;
    } else {
        return set_mode(Mode::Number::GUIDED, ModeReason::TOY_MODE);
    }
    return false;
}

bool Copter::Ugcs_do_lockon()  // lock on target
{
    if (set_mode(Mode::Number::LOCKON, ModeReason::TOY_MODE)) {
        return true;
    }
    return false;
}

bool Copter::Ugcs_do_attack()
{
    bool ret = set_mode(Mode::Number::ATTACK_ANGLE, ModeReason::TOY_MODE);
    return ret;
}

bool Copter::Ugcs_do_fs1()     // failsafe type1
{
    bool ret = set_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
    if (!ret) {
        set_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
    }
    return true;
}

void Copter::Ugcs_handle_msg(const mavlink_message_t &msg) {
    if (msg.sysid == 233) { return; }

    Location _target_location;      // last known location of target
    Vector3f _target_velocity_ned;  // last known velocity of target in NED frame in m/s
    Vector3f _target_postion_ned;
    float _target_heading = 0.0f;          // heading in degrees

    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        bool pre_is_leader = Ugcs.is_leader();
        bool is_leader = (msg.sysid == copter.g.sysid_this_mav);
        if (pre_is_leader != is_leader) {
            Ugcs.is_leader(is_leader);
            Ugcs.refresh_cmd();
        }
        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return;
        }

        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;
        _target_location.alt = packet.relative_alt / 10;        // convert millimeters to cm
        _target_location.relative_alt = 1;                // set relative_alt flag

        _target_velocity_ned.x = packet.vx * 0.01f; // velocity north
        _target_velocity_ned.y = packet.vy * 0.01f; // velocity east
        _target_velocity_ned.z = 0.0f;//packet.vz * 0.01f; // velocity down

        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
        }
        if (Ugcs.get_state() == UGround::UGCS_Assemble) {
            _target_heading = Ugcs.get_dest_yaw_cd();
        }

        // get a local timestamp with correction for transport jitter
        Ugcs._last_update_ms = millis();

        Vector2f res_vec;
        if (!_target_location.get_vector_xy_from_origin_NE(res_vec)) {return;}
        _target_postion_ned.x = res_vec.x;
        _target_postion_ned.y = res_vec.y;
        if (packet.alt > 0 && rangefinder_alt_ok() ) {
            float rng_offset = Ugcs_get_relative_alt()*0.1f - rangefinder_state.alt_cm;
            _target_postion_ned.z = rng_offset + (float)(packet.alt/10);

            //copter.gcs().send_text(MAV_SEVERITY_WARNING, "Z: %0.2f",_target_postion_ned.z);
        } else {
            _target_postion_ned.z = (float)(packet.relative_alt / 10);
        }
        Ugcs.set_up_offset(msg.sysid, _target_postion_ned, _target_velocity_ned, _target_heading);
    }
}

int32_t Copter::Ugcs_get_terrain_alt() {
    int32_t ret = -999;
    if (rangefinder_alt_ok()) {
        ret = rangefinder_state.alt_cm * 10UL;
    }
    return ret;
}

int32_t Copter::Ugcs_get_relative_alt() {
    float posD = 0.0f;
    ahrs.get_relative_position_D_home(posD);
    posD *= -1000.0f;
    return (int32_t)posD;
}

Vector3f Copter::Ugcs_get_velocity_NED() {
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }
    return vel;
}
