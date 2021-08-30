#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AP_HAL/AP_HAL.h>

#include "FD1_msg_uwb_out.h"
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
  Situational awareness for ArduPilot

 - record a series of moving points in space which should be avoided
 - produce messages for GCS if a collision risk is detected

  Peter Barker, May 2016

  based on AP_ADSB,  Tom Pittenger, November 2015
*/

#define EFGATE_NUM 11
class EF_Counter {
public:
    EF_Counter();

    static EF_Counter *get_singleton() {
        return _singleton;
    }

    /* Do not allow copies */
    EF_Counter(const EF_Counter &other) = delete;
    EF_Counter &operator=(const EF_Counter&) = delete;

    // update should be called at 10hz or higher
    void update();
    void EFGate_reset();

    FD1_msg_uwb_out& get_msg_uwb_out() {return _msg_uwb_out;}
    void uart_send(AP_HAL::UARTDriver *_port, int16_t id);

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    static EF_Counter *_singleton;

    void EFGate_update(Vector3f &pos_start, Vector3f &pos_end);
    bool EFGate_check(Vector3f &pos_start, Vector3f &pos_end, const Vector2f* lines, uint16_t num_points, uint32_t &extra_time);

    Vector3f _last_pos;

    FD1_msg_uwb_out _msg_uwb_out;

    // parameters
    AP_Float   _update_dist;
    AP_Int32   _EFGate_last_pass_time_ms[EFGATE_NUM];
};

namespace AP {
    EF_Counter &ef_counter();
};
