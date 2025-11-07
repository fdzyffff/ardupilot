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

#include "AP_RangeFinder_MUNIU.h"

#if AP_RANGEFINDER_MUNIU_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_MUNIU::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;

    // read any available lines from the lidar
    static uint32_t _last_err_post = AP_HAL::millis();
    static uint32_t _last_h_post = AP_HAL::millis();

    while (uart->available() > 0) {
        uint8_t temp = uart->read();
        _msg_ranger.parse(temp);
        if (_msg_ranger._msg_1.updated) {
            _msg_ranger._msg_1.updated = false;
            if (_msg_ranger._msg_1.content.msg.error) {
                if (AP_HAL::millis() - _last_err_post > 5000) {
                    _last_err_post = AP_HAL::millis();
                    gcs().send_text(MAV_SEVERITY_INFO, "RNGFNDER ERROR: %d ", _msg_ranger._msg_1.content.msg.error);
                }
                // break;
            }
            if (AP_HAL::millis() - _last_h_post > 6000) {
                _last_h_post = AP_HAL::millis();
                gcs().send_text(MAV_SEVERITY_INFO, "h1: %d, h2: %d, h3: %d", _msg_ranger._msg_1.content.msg.high1, _msg_ranger._msg_1.content.msg.high2, _msg_ranger._msg_1.content.msg.high3);
            }
            sum_cm += (float)_msg_ranger._msg_1.content.msg.high3;
            count++;
            // gcs().send_text(MAV_SEVERITY_INFO, "RNGFND %d",_msg_ranger._msg_1.content.msg.high3);
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_MUNIU_ENABLED
