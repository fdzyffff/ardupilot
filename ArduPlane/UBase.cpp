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

#include "Plane.h"

UBase::UBase()
{
    ;
}

// initialise
void UBase::init()
{
    _last_ms = 0;
    _active = false;
    _new_data = false;
    // _target_loc = plane.current_loc;
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.count = 0;

    _target_pos.zero();
    _target_vel.zero();
    _target_bearing = 0.0f;
}

void UBase::handle_info(Location target_loc_in, Vector3f target_pos_in, Vector3f target_vel_in, float target_heading_in)
{
    // display_info.p1 = pitch_in;
    // display_info.p2 = roll_in;
    // display_info.p3 = yaw_in;
    // display_info.p4 = abs_yaw_in;
    display_info.count++;

    _target_loc = target_loc_in;
    _target_pos = target_pos_in;
    _target_vel = target_vel_in;
    _target_bearing = target_heading_in;

    _new_data = true;
    _last_ms = millis();
}












































// support takeoff and landing on moving platforms for VTOL planes

// local PARAM_TABLE_KEY = 7
// local PARAM_TABLE_PREFIX = "SHIP_"

// local MODE_MANUAL = 0
// local MODE_RTL = 11
// local MODE_QRTL = 21
// local MODE_AUTO = 10
// local MODE_QLOITER = 19

// local NAV_TAKEOFF = 22
// local NAV_VTOL_TAKEOFF = 84

// local ALT_FRAME_ABSOLUTE = 0

// -- 3 throttle position
// local THROTTLE_LOW = 0
// local THROTTLE_MID = 1
// local THROTTLE_HIGH = 2

// // setup SHIP specific parameters
// assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
// SHIP_ENABLE     = bind_add_param('ENABLE', 1, 0)
// SHIP_LAND_ANGLE = bind_add_param('LAND_ANGLE', 2, 0)
// SHIP_AUTO_OFS   = bind_add_param('AUTO_OFS', 3, 0)

// // other parameters
// RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
// ALT_HOLD_RTL    = bind_param("ALT_HOLD_RTL")
// Q_RTL_ALT       = bind_param("Q_RTL_ALT")
// TRIM_ARSPD_CM   = bind_param("TRIM_ARSPD_CM")
// TECS_LAND_ARSPD = bind_param("TECS_LAND_ARSPD")
// Q_TRANS_DECEL   = bind_param("Q_TRANS_DECEL")
// WP_LOITER_RAD   = bind_param("WP_LOITER_RAD")
// FOLL_OFS_X      = bind_param("FOLL_OFS_X")
// FOLL_OFS_Y      = bind_param("FOLL_OFS_Y")
// FOLL_OFS_Z      = bind_param("FOLL_OFS_Z")

// an auth ID to disallow arming when we don't have the beacon
// local auth_id = arming:get_aux_auth_id()
// arming:set_aux_auth_failed(auth_id, "Ship: no beacon")

// -- current target
// local target_pos = Location()
// local current_pos = Location()
// local target_velocity = Vector3f()
// local target_heading = 0.0

// -- landing stages
// local STAGE_HOLDOFF = 0
// local STAGE_DESCEND = 1
// local STAGE_APPROACH = 2
// local STAGE_IDLE = 2
// local landing_stage = STAGE_HOLDOFF

// -- other state
// local vehicle_mode = MODE_MANUAL
// local reached_alt = false
// local throttle_pos = THROTTLE_HIGH
// local have_target = false

const struct convert_table {
    const char* p_name;
    float new_val;
}  conversion_table[] = {
    // PARAMETER_CONVERSION - Added: Aug-2021
    {"FOLL_ENABLE", 1},
    {"FOLL_OFS_TYPE", 1},
    {"FOLL_ALT_TYPE", 1},
};

// check key parameters
void UBase::check_parameters()
{
    for (const auto & elem : conversion_table) {
        float val = 0.0f;
        if (AP_Param::get(elem.p_name, val)) {
            AP_Param::set_by_name(elem.p_name, elem.new_val);
            gcs().send_text(MAV_SEVERITY_INFO,"Parameter %s set to %.2f was %.2f", elem.p_name, elem.new_val, val);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO,"Parameter %s not found", elem.p_name);
        }
    }
}

// update the pilots throttle position
void UBase::update_throttle_pos()
{
    throttle_pos t_throttle_pos = throttle_pos::LOW;
    if (!rc().has_valid_input()) {
        t_throttle_pos = throttle_pos::LOW;
        return;
    }
    else {
        const RC_Channel *tchan = rc().channel(RCMAP_THROTTLE.get());
        if (tchan == nullptr) {
            t_throttle_pos = throttle_pos::LOW;
            return;
        }
        float tval = (tchan->norm_input_ignore_trim()+1.0f)*0.5f;
        if (tval >= 0.4f) {
            t_throttle_pos = throttle_pos::HIGH
        }
        else if (tval >= 0.1f) {
            t_throttle_pos = throttle_pos::MID;
        }
        else {
            t_throttle_pos = throttle_pos::LOW;
        }
    }
    if (t_throttle_pos != mlstate.throttle_pos) {
        mlstate.reached_alt = false;
        if (mlstate.landing_stage == land_stage::HOLDOFF && t_throttle_pos <= throttle_pos::MID) {
            mlstate.landing_stage = land_stage::DESCEND;
            gcs().send_text(MAV_SEVERITY_INFO, "Descending for approach (hd=%.1fm h=%.1f th=%.1f)", get_holdoff_distance(), current_pos:alt()*0.01, get_target_alt());
        }
        if (mlstate.landing_stage == land_stage::DESCEND && t_throttle_pos == throttle_pos::HIGH) {
            gcs:send_text(MAV_SEVERITY_INFO, "Climbing for holdoff");
            mlstate.landing_stage = land_stage::HOLDOFF;
        }
        mlstate.throttle_pos = t_throttle_pos;
    }
}

// get landing airspeed
float UBase:: get_land_airspeed()
{
    if (TECS_LAND_ARSPD.get() < 0.0f ) {
        return TRIM_ARSPD_CM.get() * 0.01f;
    }
    return TECS_LAND_ARSPD.get();
}

//   calculate stopping distance assuming we are flying at
//   TECS_LAND_ARSPD and are approaching the landing target from
//   behind. Take account of the wind estimate to get approach
//   groundspeed
float UBase::stopping_distance()
{
    // get the target true airspeed for approach
    float tas = get_land_airspeed() * ahrs().get_EAS2TAS();

    // add in wind in direction of flight
    Vector2f wind = ahrs().wind_estimate().xy();

    // rotate wind to be in approach frame
    wind.rotate(-radians(mlstate.target_heading + SHIP_LAND_ANGLE.get())); 

    // ship velocity rotated to the approach frame
    Vector2f ship2d = target_velocity:xy();
    ship2d.rotate(-radians(mlstate.target_heading + SHIP_LAND_ANGLE.get()));

    // calculate closing speed
    // use pythagoras theorem to solve for the wind triangle
    float tas_sq = sq(tas);
    float y_sq = sq(wind.y);
    float closing_speed = 1.0f;
    if (tas_sq >= y_sq) {
       closing_speed = sqrt(tas_sq - y_sq);
    }
    else {
       // min 1 m/s
       closing_speed = 1.0f;
    }

    // include the wind in the direction of the ship
    closing_speed = closing_speed + wind.x; 

    // account for the ship velocity
    closing_speed = closing_speed - ship2d.x; 

    // calculate stopping distance
    return sq(closing_speed) / (2.0f * Q_TRANS_DECEL.get());
}

// get holdoff distance
float UBase::get_holdoff_distance()
{
   float radius = WP_LOITER_RAD.get();
   float holdoff_dist = fabsf(radius*1.5f);
   float stop_distance = stopping_distance();

   // increase holdoff distance by up to 50% to ensure we can stop
   holdoff_dist = MAX(holdoff_dist, MIN(holdoff_dist*2.5f, stop_distance*2.0f));
   return holdoff_dist;
}

// get the holdoff position
Location UBase::get_holdoff_position()
{
   float radius = WP_LOITER_RAD.get();
   float heading_deg = mlstate.target_heading + SHIP_LAND_ANGLE.get();
   float holdoff_dist = get_holdoff_distance();

   Vector2f ofs = Vector2f(-holdoff_dist, radius);
   ofs.rotate(radians(heading_deg));
   Location target = mlstate.target_pos;
   target.offset(ofs.xy());
   return target;
}

// check if we have reached the tangent to the landing location
bool UBase::check_approach_tangent()
{
    float distance = mlstate.current_pos.get_distance(mlstate.target_pos);
    float holdoff_dist = get_holdoff_distance();
    if (mlstate.landing_stage == land_state::HOLDOFF and mlstate.throttle_pos <= throttle_pos::MID && distance < 4.0f*holdoff_dist) {
        gcs().send_text(MAV_SEVERITY_INFO, "Descending for approach (hd=%.1fm)", holdoff_dist);
        mlstate.landing_stage = land_state::DESCEND;
    }
    if (mlstate.reached_alt && mlstate.landing_stage == land_state::DESCEND) {
        // go to approach stage when throttle is low, we are
        // pointing at the ship and have reached target alt.
        // Also require we are within 2.5 radius of the ship, and our heading is within 20
        // degrees of the target heading
        float target_bearing_deg = wrap_180(degrees(mlstate.current_pos.get_bearing(mlstate.target_pos)));
        float ground_bearing_deg = wrap_180(degrees(ahrs().groundspeed_vector().angle()));
        float margin = 10.0f;
        float distance = mlstate.current_pos.get_distance(mlstate.target_pos);
        float holdoff_dist = get_holdoff_distance();
        float error1 = fabsf(wrap_180(target_bearing_deg - ground_bearing_deg));
        float error2 = fabsf(wrap_180(ground_bearing_deg - (mlstate.target_heading + SHIP_LAND_ANGLE.get())));
        logger().write('SLND','TBrg,GBrg,Dist,HDist,Err1,Err2','ffffff',target_bearing_deg, ground_bearing_deg, distance, holdoff_dist, error1, error2);
        if (error1 < margin &&
             distance < 2.5*holdoff_dist &&
             distance > 0.7*holdoff_dist &&
             error2 < 2*margin) {
            // we are on the tangent, switch to QRTL
            gcs:send_text(0, "Starting approach");
            landing_stage = STAGE_APPROACH;
            vehicle:set_mode(MODE_QRTL);
        }
    }
}

// check if we should abort a QRTL landing
void UBase::check_approach_abort()
{
    float alt = mlstate.current_pos.alt * 0.01f;
    float target_alt = get_target_alt();
    if (alt > target_alt) {
        gcs().send_text(MAV_SEVERITY_INFO, "Aborting landing");
        mlstate.landing_stage = land_stage::HOLDOFF;
        plane.set_mode(plane.mode_rtl, ModeReason::AUTO_RTL_EXIT);
    }
}

// update state based on vehicle mode
void UBase::update_mode()
{
    uint8_t mode = plane.get_mode();
    if (mode == mlstate.vehicle_mode) {
        return;
    }
    mlstate.vehicle_mode = mode;
    if (mlstate.vehicle_mode == MODE_RTL) {
        mlstate.landing_stage = land_stage::HOLDOFF;
        mlstate.reached_alt = false;
    } else if (mlstate.vehicle_mode != MODE_RTL) {
        mlstate.landing_stage = land_stage::IDLE;
        mlstate.reached_alt = false;
    }
}

// update target state
void UBase::update_target()
{
    if (!g2.follow.have_target()) {
        if (mlstate.have_target) {
            gcs().send_text(MAV_SEVERITY_INFO,"Lost beacon");
            arming:set_aux_auth_failed(auth_id, "Ship: no beacon");
        }
        mlstate.have_target = false;
    }
    else {
        if (!mlstate.have_target) {
            gcs().send_text(MAV_SEVERITY_INFO,"Have beacon");
            arming:set_aux_auth_passed(auth_id);
        }
        mlstate.have_target = true;
        g2.follow.get_target_location_and_velocity_ofs(mlstate.target_pos, mlstate.target_velocity);
        mlstate.target_pos.change_alt_frame(Location::AltFrame::ALT_FRAME_ABSOLUTE);
        // zero vertical velocity to reduce impact of ship movement
        mlstate.target_velocity.z = 0.0f;
    }
}

// get the alt target for holdoff, AMSL
float UBase::get_target_alt()
{
    float base_alt = mlstate.target_pos.alt * 0.01f;
    if (mlstate.landing_stage == land_stage::HOLDOFF) {
        return base_alt + ALT_HOLD_RTL.get() * 0.01f;
    }
    return base_alt + Q_RTL_ALT.get();
}

void UBase::update_alt()
{
    float alt = mlstate.current_pos.alt * 0.01;
    float target_alt = get_target_alt();
    if (mlstate.landing_stage == land_state::HOLDOFF || mlstate.landing_stage == land_stage::DESCEND) {
        if (fabsf(alt - target_alt) < 3.0f) {
            if (!mlstage.reached_alt) {
                gcs().send_text(MAV_SEVERITY_INFO, "Reached target altitude");
            }
            mlstate.reached_alt = true;
        }
    }
}

// update automatic beacon offsets
void UBase::update_auto_offset()
{
    if (arming::is_armed() || (SHIP_AUTO_OFS.get() != 1)) {
       return;
    }

    // get target without offsets applied
    Vector3f target_no_ofs;
    Vector3f vel;
    if (!g2.follow.get_target_location_and_velocity(target_no_ofs, vel)) {
        return;
    }

    // setup offsets so target location will be current location
    Vector3f new_offs = target_no_ofs.get_distance_NED(mlstate.current_pos);
    new_offs.rotate_xy(-radians(mlstate.target_heading));

    gcs:send_text(MAV_SEVERITY_INFO, "Set follow offset (%.2f,%.2f,%.2f)", new_offs:x(), new_offs:y(), new_offs:z());
    FOLL_OFS_X:set_and_save(new_offs.x);
    FOLL_OFS_Y:set_and_save(new_offs.y);
    FOLL_OFS_Z:set_and_save(new_offs.z);

    SHIP_AUTO_OFS.set_and_save(0);
}

// update
void UBase::update()
{
    if (!_enable) {
        return;
    }
    update_target();
    if (!_have_target) {
        return;
    }

    mlstate.current_pos = plane.current_loc;
    mlstate.current_pos.change_alt_frame(Location::AltFrame::ALT_FRAME_ABSOLUTE);

    update_throttle_pos();
    update_mode();
    update_alt();
    update_auto_offset();

    plane.ahrs.set_home(mlstate.target_pos);

    Location next_WP;
    if (!plane.get_target_location(next_WP)) {
        // not in a flight mode with a target location
        return;
    }

    if (mlstate.vehicle_mode == Mode::Number::MODE_RTL) {
        Location holdoff_pos = get_holdoff_position();
        holdoff_pos.set_alt_cm(get_target_alt()*100.f, Location::AltFrame::ALT_FRAME_ABSOLUTE);
        plane.update_target_location(next_WP, holdoff_pos);

        if (mlstate.throttle_pos == THROTTLE_LOW) {
            check_approach_tangent();
        }
    }
    else if (mlstate.vehicle_mode == Mode::Number::MODE_QRTL) {
        plane.set_velocity_match(mlstate.target_velocity.xy());
        Location t_target_pos = current_pos;
        t_target_pos.set_alt_cm(next_WP.get_alt_frame(), next_WP.alt);
        plane.update_target_location(next_WP, t_target_pos);

        if (mlstate.throttle_pos == THROTTLE_HIGH) {
            check_approach_abort();
        }
    }
    else if (mlstate.vehicle_mode == Mode::Number::MODE_AUTO) {
        uint16_t id = plane.mission.get_current_nav_id();
        if (id == NAV_VTOL_TAKEOFF || id == NAV_TAKEOFF) {
            plane.set_velocity_match(mlstate.target_velocity.xy());
            Location t_target_pos = current_pos;
            t_target_pos.set_alt_cm(next_WP.get_alt_frame(), next_WP.alt);
            vehicle:update_target_location(next_WP, t_target_pos);
        }
        else if (id == MAV_CMD_NAV_LAND || id == MAV_CMD_NAV_VTOL_LAND) {
            Location t_target_pos = current_pos;
            t_target_pos.set_alt_cm(next_WP.get_alt_frame(), next_WP.alt);
            plane.update_target_location(next_WP, t_target_pos);
        }
    }
    else if (mlstate.vehicle_mode == Mode::Number::MODE_QLOITER) {
        plane.set_velocity_match(mlstate.target_velocity.xy());
    }
}
