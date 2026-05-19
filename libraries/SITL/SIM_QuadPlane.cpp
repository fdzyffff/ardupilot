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
/*
  simple quadplane simulator class
*/

#include "SIM_QuadPlane.h"

#include <stdio.h>

using namespace SITL;

QuadPlane::QuadPlane(const char *frame_str) :
    Plane(frame_str)
{
    // default to X frame
    const char *frame_type = "x";
    uint8_t motor_offset = 4;

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    if (strstr(frame_str, "-octa-quad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octaquad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octa")) {
        frame_type = "octa";
    } else if (strstr(frame_str, "-hexax")) {
        frame_type = "hexax";
    } else if (strstr(frame_str, "-hexa")) {
        frame_type = "hexa";
    } else if (strstr(frame_str, "-plus")) {
        frame_type = "+";
    } else if (strstr(frame_str, "-y6")) {
        frame_type = "y6";
    } else if (strstr(frame_str, "-tri")) {
        frame_type = "tri";
    } else if (strstr(frame_str, "-tilttrivec")) {
        frame_type = "tilttrivec";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-tilthvec")) {
        frame_type = "tilthvec";
    } else if (strstr(frame_str, "-tilttri")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "firefly")) {
        frame_type = "firefly";
        // elevon style surfaces
        elevons = true;
        // fwd motor gives zero thrust
        thrust_scale = 0;
        // vtol motors start at 2
        motor_offset = 2;
    } else if (strstr(frame_str, "-tilt")) {
        frame_type = "tilt";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "cl84")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-copter_tailsitter")) {
        frame_type = "+";
        copter_tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    frame = Frame::find_frame(frame_type);
    if (frame == nullptr) {
        printf("Failed to find frame '%s'\n", frame_type);
        exit(1);
    }

    if (strstr(frame_str, "cl84")) {
        // setup retract servos at front
        frame->motors[0].servo_type = Motor::SERVO_RETRACT;
        frame->motors[0].servo_rate = 7*60.0/90; // 7 seconds to change
        frame->motors[1].servo_type = Motor::SERVO_RETRACT;
        frame->motors[1].servo_rate = 7*60.0/90; // 7 seconds to change
    }
    
    // leave first 4 servos free for plane
    frame->motor_offset = motor_offset;

    // we use zero terminal velocity to let the plane model handle the drag
    frame->init(frame_str, &battery);

    AP_Param::load_object_from_eeprom(sitl, sitl->var_infosimparam);
    if (sitl) {
        coefficient.s = sitl->s;
        coefficient.b = sitl->b;
        coefficient.c = sitl->c;
        coefficient.c_lift_0 = sitl->c_lift_0;
        coefficient.c_lift_deltae = sitl->c_lift_deltae;
        coefficient.c_lift_a = sitl->c_lift_a;
        coefficient.c_lift_q = sitl->c_lift_q;
        coefficient.mcoeff = sitl->mcoeff;
        coefficient.oswald = sitl->oswald;
        coefficient.alpha_stall = sitl->alpha_stall;
        coefficient.c_drag_q = sitl->c_drag_q;
        coefficient.c_drag_deltae = sitl->c_drag_deltae;
        coefficient.c_drag_p = sitl->c_drag_p;
        coefficient.c_y_0 = sitl->c_y_0;
        coefficient.c_y_b = sitl->c_y_b;
        coefficient.c_y_p = sitl->c_y_p;
        coefficient.c_y_r = sitl->c_y_r;
        coefficient.c_y_deltaa = sitl->c_y_deltaa;
        coefficient.c_y_deltar = sitl->c_y_deltar;
        coefficient.c_l_0 = sitl->c_l_0;
        coefficient.c_l_p = sitl->c_l_p;
        coefficient.c_l_b = sitl->c_l_b;
        coefficient.c_l_r = sitl->c_l_r;
        coefficient.c_l_deltaa = sitl->c_l_deltaa;
        coefficient.c_l_deltar = sitl->c_l_deltar;
        coefficient.c_m_0 = sitl->c_m_0;
        coefficient.c_m_a = sitl->c_m_a;
        coefficient.c_m_q = sitl->c_m_q;
        coefficient.c_m_deltae = sitl->c_m_deltae;
        coefficient.c_n_0 = sitl->c_n_0;
        coefficient.c_n_b = sitl->c_n_b;
        coefficient.c_n_p = sitl->c_n_p;
        coefficient.c_n_r = sitl->c_n_r;
        coefficient.c_n_deltaa = sitl->c_n_deltaa;
        coefficient.c_n_deltar = sitl->c_n_deltar;
        coefficient.deltaa_max = sitl->deltaa_max;
        coefficient.deltae_max = sitl->deltae_max;
        coefficient.deltar_max = sitl->deltar_max;
        coefficient.CGOffset.x = sitl->CGOffset_x;
        coefficient.CGOffset.y = sitl->CGOffset_y;
        coefficient.CGOffset.z = sitl->CGOffset_z;
        mass = sitl->mass;
        thrust_scale = sitl->thrust_scale;
        have_drop = sitl->have_drop;
        ::printf("Load plane sim param\n");
    }

    // increase mass for plane components
    frame->set_mass(mass * 1.5f);
    // mass = frame->get_mass() * 1.5;
    // frame->set_mass(mass);

    lock_step_scheduled = true;
}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;

    motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body, rpm, false);

    // rotate frames for copter tailsitters
    if (copter_tailsitter) {
        quad_rot_accel.rotate(ROTATION_PITCH_270);
        quad_accel_body.rotate(ROTATION_PITCH_270);
    }

    // estimate voltage and current
    frame->current_and_voltage(battery_voltage, battery_current);

    battery.set_current(battery_current);

    float throttle;
    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    // assume 20A at full fwd throttle
    throttle = fabsf(throttle);
    battery_current += 20 * throttle;

    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
