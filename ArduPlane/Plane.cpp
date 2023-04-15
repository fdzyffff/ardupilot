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

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane plane;
AP_Vehicle& vehicle = plane;



// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Plane::position_ok() 
{
    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Plane::ekf_has_absolute_position() 
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal position
    if (!arming.is_armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Plane::ekf_has_relative_position() 
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal relative position
    if (!arming.is_armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}
