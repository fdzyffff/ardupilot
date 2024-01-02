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
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

MultiCopter::MultiCopter(const char *frame_str) :
    Aircraft(frame_str)
{
    frame = Frame::find_frame(frame_str);
    if (frame == nullptr) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }

    frame->init(frame_str, &battery);

    mass = frame->get_mass();
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;

    // if (strstr(frame_str, "-throw")) {
    //     have_launcher = true;
    //     launch_start_ms = 0;
    // } else {
    //     have_launcher = false;
    //     launch_start_ms = 0;
    // }
    have_launcher = true;
    launch_start_ms = 0;
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    frame->calculate_forces(*this, input, rot_accel, body_accel, rpm);

    bool launch_triggered = input.servos[8] > 1700;//channel No.9
    if (have_launcher) {
        uint32_t launch_time = 1;
        float launch_accel = 7.0f;
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                printf("launch triggered %f\n",accel_body.z);
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                accel_body.x += mass * launch_accel/10;
                accel_body.z -= mass * launch_accel;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }

    add_shove_forces(rot_accel, body_accel);
    add_twist_forces(rot_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    // estimate voltage and current
    frame->current_and_voltage(battery_voltage, battery_current);

    battery.set_current(battery_current);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

