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
    _group_distance = 200.f;
    _cmd = 0;
    _follow_loc_vec.zero();
    _follow_vel_vec.zero();
    _follow_yaw_cd = 0.0f;
    _dest_loc_vec.zero();
    _dest_loc_vec.z = get_final_target_alt();
    _raw_dest_loc_vec.zero();
    _dist_to_target = 0.0f;
    _bearing_to_target = 0.0f;
    _yaw_middle_cd = 0.0f;
    _group_target_yaw = (float)(copter.ahrs.yaw_sensor/100);
    _group_current_yaw = (float)(copter.ahrs.yaw_sensor/100);
    _new_dist = false;
    _leader_id = copter.g.sysid_this_mav;
    _leader_loc_vec.zero();
    _gcs_target_alt_offset = 0.0f;
}

void UGround::handle_info(int16_t p1, float p2, float p3, float p4)
{
    switch (p1) {
        case 1:
            do_cmd((int16_t)p2);
            break;
        case 2:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Set up group");
            set_up_group( (int16_t)p2, p3, p4);
            break;
        case 3:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Set up alt");
            set_up_alt(p2);
            break;
        case 4:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Set up search dist: %f",p2);
            set_up_search_dist(p2);
            break;
        case 5:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Set up alt offset: %f",p2);
            set_up_alt_offset(p2);
            break;
        case 10:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Set up dest :%f, %f",p2, p3);
            set_up_dest(p2, p3);
            break;
        default:
            break;
    }
}

void UGround::do_cmd(int16_t cmd) {
    switch (cmd) {
        case 1:
            do_takeoff();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do takeoff");
            _cmd = cmd;
            break;
        case 2:
            do_fly();
            _cmd = cmd;
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do fly");
            break;
        case 3:
            do_search();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do search");
            _cmd = cmd;
            break;
        case 4:
            do_assemble();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do assemble [%d]", is_active());
            _cmd = cmd;
            break;
        case 5:
            do_attack();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do attack");
            _cmd = cmd;
            break;
        case 6:
            do_fs1();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do fs1");
            _cmd = cmd;
            break;
        case 7:
            do_pause();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do pause");
            break;
        case 10:
            copter.Ucam.do_cmd(1.0f);
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "CAM ON");
            break;
        case 11:
            copter.Ucam.do_cmd(0.0f);
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "CAM OFF");
            break;
        case 12:
            copter.Ucam.do_cmd(10.0f);
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Plot get");
            break;
        case 254:
            do_arm();
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Do arm");
            break;
        case 255:
            copter.arming.disarm(AP_Arming::Method::MAVLINK, false);
            break;
        default:
            break;
    }
}

void UGround::set_up_group( int16_t group_id, float distance, float direction) {
    _group_id = group_id;
    _group_target_yaw = direction;
    if (distance > 10.f) {
        _group_distance = distance;
    }
    refresh_dest();
}

void UGround::set_up_alt(float target_alt) {
    copter.g2.user_parameters.gcs_target_alt.set(target_alt);
    refresh_dest();
}

void UGround::set_up_alt_offset(float target_alt_offset) {
    _gcs_target_alt_offset += target_alt_offset;
    refresh_dest();
}

void UGround::clear_up_alt_offset() {
    _gcs_target_alt_offset = 0.0f;
    refresh_dest();
}

void UGround::set_up_search_dist(float search_dist) {
    copter.g2.user_parameters.group_search_dist.set(search_dist);
}

float UGround::get_final_target_alt() {
    return ((float)copter.g2.user_parameters.gcs_target_alt.get()+_gcs_target_alt_offset);
}

void UGround::refresh_dest() {
    // offset from virtual center point to this vehicle
    Vector3f offset_position = get_offset(copter.g.sysid_this_mav, 0, _group_distance);
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(_group_target_yaw));
    offset_position = tmp_m*offset_position;
    _dest_loc_vec = _raw_dest_loc_vec+offset_position;
    _dest_loc_vec.z = get_final_target_alt();

    gcs().send_text(MAV_SEVERITY_WARNING, "[x,y,z]: [%f,%f,%f]", _dest_loc_vec.x,_dest_loc_vec.y,_dest_loc_vec.z);

    dest_pos_update(true);
}

void UGround::set_up_dest(float lat_in, float lng_in) {
    Location tmp_location;
    tmp_location.lat = (int32_t)(lat_in*1.0e7f);
    tmp_location.lng = (int32_t)(lng_in*1.0e7f);
    tmp_location.alt = (int16_t)get_final_target_alt();
    tmp_location.relative_alt = 1; 

    gcs().send_text(MAV_SEVERITY_WARNING, "lng: %f", (float)tmp_location.lng);
    gcs().send_text(MAV_SEVERITY_WARNING, "lat: %f", (float)tmp_location.lat);
    gcs().send_text(MAV_SEVERITY_WARNING, "alt: %f", (float)tmp_location.alt);

    Vector2f res_vec;
    if (!tmp_location.get_vector_xy_from_origin_NE(res_vec)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "Failed setup dest");
        return;
    }
    Vector3f target_postion = Vector3f(res_vec.x, res_vec.y, get_final_target_alt());
    Vector3f new_raw_dest_loc_vec = target_postion;
    if (norm(new_raw_dest_loc_vec.x - _raw_dest_loc_vec.x, new_raw_dest_loc_vec.y - _raw_dest_loc_vec.y) > 200.f) {
        _raw_dest_loc_vec = new_raw_dest_loc_vec;
        refresh_dest();
    }
    // _raw_dest_loc_vec = target_postion;
}

void UGround::refresh_cmd() {
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "(%d)cmd:%d",_is_leader,_cmd);
    do_cmd(_cmd);
}

void UGround::set_up_follow(int8_t sender_id, Vector3f target_postion, Vector3f target_velocity, float target_heading) {
    Vector3f offset_position = get_offset(copter.g.sysid_this_mav, sender_id, _group_distance);
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(_group_current_yaw));
    offset_position = tmp_m*offset_position;

    Vector3f vel_comp = Vector3f(target_velocity.x*copter.g2.user_parameters.gcs_group_delay.get()*0.001f, target_velocity.y*copter.g2.user_parameters.gcs_group_delay.get()*0.001f, 0.0f);
    // Vector3f vel_comp = Vector3f(target_velocity.x*1.0f, target_velocity.y*1.0f, 0.0f);

    if (norm(target_velocity.x, target_velocity.y) < 200.f) {
        vel_comp.zero();
    }

    // Vector3f new_vec = target_postion + offset_position + vel_comp;
    // if (norm(new_vec.x-_follow_loc_vec.x, new_vec.y-_follow_loc_vec.y) > 200.f) {
        _follow_loc_vec = target_postion + offset_position + vel_comp;
    // }
    _follow_vel_vec = target_velocity;
    _follow_yaw_cd = target_heading*100.f;
    _leader_id = sender_id;
    _leader_loc_vec = target_postion + vel_comp;
}

void UGround::clean_follow() {
    _follow_vel_vec.zero();
}

void UGround::update_group_yaw(float dt) {
    float yaw_rate_limit = 500.f/MAX(500.f, _group_distance*4.0f)*57.f;
    float _delta_yaw = constrain_float(wrap_180(_group_target_yaw - _group_current_yaw), -yaw_rate_limit*dt, yaw_rate_limit*dt);
    _group_current_yaw = wrap_360(_group_current_yaw + _delta_yaw);
}

Vector3f UGround::get_search_dest() {
    Vector3f search_offset_position;
    switch (_group_id) {
    case 1:
        search_offset_position = my_group1.get_search_dest(copter.g.sysid_this_mav, _group_distance, copter.g2.user_parameters.group_search_dist.get());
        break;
    case 2:
        search_offset_position = my_group2.get_search_dest(copter.g.sysid_this_mav, _group_distance, 500.f);
        break;
    default:
        break;
    }

    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(_group_current_yaw));
    search_offset_position = tmp_m*search_offset_position;

    return (get_dest_loc_vec()+search_offset_position);
}

Vector3f UGround::get_assemble_dest() {
    Vector3f assemble_offset_position = my_group1_assemble.get_offset(copter.g.sysid_this_mav, _leader_id, 1000.f);

    float group_dir = wrap_360(_follow_yaw_cd*0.01f - my_group1_assemble.get_dir(_leader_id)); // in degree
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(group_dir));
    assemble_offset_position = tmp_m*assemble_offset_position;
    _leader_loc_vec.z = (float)(get_final_target_alt());
    // update _yaw_middle_cd to possible target dir
    _yaw_middle_cd = wrap_360(_follow_yaw_cd*0.01f + my_group1_assemble.get_dir(copter.g.sysid_this_mav) - my_group1_assemble.get_dir(_leader_id))*100.0f;
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "dir: %f",_yaw_middle_cd);
    return (_leader_loc_vec+assemble_offset_position);
}

Vector3f UGround::get_offset(int8_t id_A, int8_t id_B, float distance) {
    Vector3f offset_position;
    offset_position.zero();
    switch (_group_id) {
        case 1:
            offset_position = my_group1.get_offset(id_A, id_B, distance);
            break;
        case 2:
            offset_position = my_group2.get_offset(id_A, id_B, distance);
            break;
        default:
            break;
    }
    return offset_position;
}

// update
void UGround::update()
{
    update_group_yaw(0.02f);
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.gcs_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _active = false;
        clean_follow();
    } else {
        _active = true;
    }
}

int16_t UGround::get_state_num() {
    int16_t ret = -1;
    if (is_leader()) {
        switch (copter.flightmode->mode_number()) {
            case Mode::Number::TAKEOFF:
                ret = 1;
                break;
            case Mode::Number::FLY:
                ret = 2;
                break;
            case Mode::Number::SEARCH:
                ret = 4;
                break;
            case Mode::Number::LOCKON:
            case Mode::Number::ASSEMBLE:
                ret = 5;
                break;
            case Mode::Number::ATTACK_ANGLE:
                ret = 6;
                break;
            case Mode::Number::LAND:
                ret = 7;
                break;
            default:
                ret = 0;
                break;
        }
    } else {
        switch (copter.flightmode->mode_number()) {
            case Mode::Number::TAKEOFF:
                ret = 1;
                break;
            case Mode::Number::FLY:
                ret = 3;
                break;
            case Mode::Number::ASSEMBLE:
            case Mode::Number::LOCKON:
                ret = 5;
                break;
            case Mode::Number::ATTACK_ANGLE:
                ret = 6;
                break;
            case Mode::Number::LAND:
                ret = 7;
                break;
            default:
                ret = 0;
                break;
        }
    }
    return ret;
}


float UGround::get_lockon_yaw_rate() {
    return get_lockon_yaw_rate(_yaw_middle_cd);
}

float UGround::get_lockon_yaw_rate(float yaw_middle_cd) {
    if (copter.flightmode->mode_number() != Mode::Number::LOCKON) {return 0.0f;}
    float para_angle_right = constrain_float(copter.g2.user_parameters.gcs_search_yangle_right * 100.f, 0.0f, 18000.f);
    float para_angle_left = constrain_float(copter.g2.user_parameters.gcs_search_yangle_left * 100.f, -18000.f, para_angle_right);
    float para_angle_rate = fabsf(copter.g2.user_parameters.gcs_search_yrate) * 100.f;
    static uint32_t _last_ms = millis();
    static float angle_rate = para_angle_rate;
    uint32_t now = millis();
    float delta_cd = wrap_180_cd(copter.attitude_control->get_att_target_euler_cd().z - yaw_middle_cd);
    if (now - _last_ms > 1000) {
        angle_rate = para_angle_rate;
    } else {
        if (delta_cd < para_angle_left) {
            angle_rate = para_angle_rate;
        } else if (delta_cd > para_angle_right) {
            angle_rate = -para_angle_rate;
        }
    }
    _last_ms = now;
    return angle_rate;
}

bool UGround::do_arm()
{
    if (copter.motors->armed()) {
        return true;
    }

    copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND);
    if (copter.arming.arm(AP_Arming::Method::MAVLINK)) {
        return true;
    }
    return false;
}

bool UGround::do_takeoff() // takeoff
{
    if (!copter.motors->armed()) {
        return do_arm();
    }

    if (copter.set_mode(Mode::Number::TAKEOFF, ModeReason::GCS_COMMAND)) {
        return copter.mode_guided.do_user_takeoff(constrain_float(copter.g.pilot_takeoff_alt,100.0f,1500.0f),true);
    }

    // if (copter.set_mode(Mode::Number::LOITERTKOFF, ModeReason::GCS_COMMAND)) {
    //     copter.mode_loitertkoff.set_climb_rate(20.f);
    //     return true;
    // }
    return false;
}

bool UGround::do_fly()     // fly
{
    refresh_dest();
    return copter.set_mode(Mode::Number::FLY, ModeReason::GCS_COMMAND);
}

bool UGround::do_search()  // search
{
    return copter.set_mode(Mode::Number::SEARCH, ModeReason::GCS_COMMAND);
}

bool UGround::do_assemble()  // assemble
{
    if(is_leader() || !is_active()) {
        _yaw_middle_cd = copter.ahrs.yaw_sensor;
        return copter.set_mode(Mode::Number::LOCKON, ModeReason::TOY_MODE);
    } else {
        _yaw_middle_cd = _follow_yaw_cd;
        return copter.set_mode(Mode::Number::ASSEMBLE, ModeReason::GCS_COMMAND);
    }
}

bool UGround::do_lockon()  // lock on target
{
    return copter.set_mode(Mode::Number::LOCKON, ModeReason::TOY_MODE);
}

bool UGround::do_attack()
{
    return copter.set_mode(Mode::Number::ATTACK_ANGLE, ModeReason::TOY_MODE);
}

bool UGround::do_fs1()     // failsafe type1
{
    bool ret = copter.set_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
    if (!ret) {
        copter.set_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
    }
    return true;
}

bool UGround::do_pause()
{
    return copter.set_mode(Mode::Number::BRAKE, ModeReason::TOY_MODE);
}

//~~~~~~~~~~~~~~~~~~~~~~~ Copter ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Copter::Ugcs_handle_msg(const mavlink_message_t &msg) {
    if (msg.sysid == 233) { return; }

    Location _target_location;      // last known location of target
    Vector3f _target_velocity_neu;  // last known velocity of target in NED frame in m/s
    Vector3f _target_postion_neu;
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
        _target_location.alt = packet.relative_alt / 10;  // convert millimeters to cm
        _target_location.relative_alt = 1;                // set relative_alt flag

        _target_velocity_neu.x = 0.0f; //packet.vx*0.1f; // velocity north
        _target_velocity_neu.y = 0.0f; //packet.vy*0.1f; // velocity east
        _target_velocity_neu.z = 0.0f; //packet.vz * 0.01f; // velocity down

        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
        }

        // get a local timestamp with correction for transport jitter
        Ugcs._last_update_ms = millis();

        Vector2f res_vec;
        if (!_target_location.get_vector_xy_from_origin_NE(res_vec)) {return;}
        _target_postion_neu.x = res_vec.x;
        _target_postion_neu.y = res_vec.y;
        
        float rng_offset = 0.0f;
        if (rangefinder_alt_ok() ) {
            rng_offset = Ugcs_get_relative_alt()*0.1f - rangefinder_state.alt_cm;
        }
        _target_postion_neu.z = rng_offset + (float)(Ugcs.get_final_target_alt());

        Ugcs.set_up_follow(msg.sysid, _target_postion_neu, _target_velocity_neu, _target_heading);
    }
}

int32_t Copter::Ugcs_get_terrain_target_alt() {
    return Ugcs.get_final_target_alt();
}

int32_t Copter::Ugcs_get_relative_alt() {
    float posD = 0.0f;
    // ahrs.get_relative_position_D_home(posD);
    if (ahrs.get_relative_position_D_origin(posD)) {
        posD *= -1000.0f;
    }
    return (int32_t)posD;
}

Vector3f Copter::Ugcs_get_velocity_NED() {
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }
    return vel;
}

float Copter::Ugcs_get_target_yaw_cd() {
    return attitude_control->get_att_target_euler_cd().z;
}

Location Copter::Ugcs_get_target_pos_location() {
    Location originLLH;
    if (!ahrs.get_origin(originLLH) || !ahrs.home_is_set()) {
        return current_loc;
    }
    const Vector3f offset = originLLH.get_distance_NED(ahrs.get_home());
    Vector3p pos_target = pos_control->get_pos_target_cm(); // to home
    pos_target.x += offset.x; 
    pos_target.y += offset.y; // target pos to origin

    Location ret = Location(pos_target, Location::AltFrame::ABOVE_ORIGIN);
    return ret;
}
