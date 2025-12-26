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

#include "AP_RangeFinder_NRA15.h"

#if AP_RANGEFINDER_NRA15_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define NRA15_FRAME_HEADER 0xAA
#define NRA15_FRAME_LENGTH 14
#define NRA15_DIST_MAX_CM 32768
#define NRA15_OUT_OF_RANGE_ADD_CM 100
#define NRA15_FRAME_END 0x55

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_NRA15::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // gcs().send_text(MAV_SEVERITY_INFO, "get_reading");

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read any available lines from the lidar
    for (auto j=0; j<8192; j++) {
        uint8_t c;
        // uart->write(0x99);
        if (!uart->read(c)) {
            break;
        }
        // if buffer is empty and this byte is 0x59, add to buffer
        if (linebuf_len == 0) {
            if (c == NRA15_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x59, add it to buffer
            // if not clear the buffer
            if (c == NRA15_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 10 items try to decode it
            if (linebuf_len >= (NRA15_FRAME_LENGTH)) {
                // if end matches extract contents
                if (NRA15_FRAME_END == linebuf[NRA15_FRAME_LENGTH-1] && NRA15_FRAME_END == linebuf[NRA15_FRAME_LENGTH-2]) {
                    // calculate distance
                    uint16_t dist = ((uint16_t)linebuf[4] << 8) | linebuf[5];
                    if (dist >= NRA15_DIST_MAX_CM || dist == uint16_t(model_dist_max_cm())) {
                        // this reading is out of range. Note that we
                        // consider getting exactly the model dist max
                        // is out of range. This fixes an issue with
                        // the TF03 which can give exactly 18000 cm
                        // when out of range
                        count_out_of_range++;
                    } else if (!has_signal_byte()) {
                        // no signal byte from TFmini so add distance to sum
                        sum_cm += dist;
                        count++;
                    } else {
                        // add distance to sum
                        sum_cm += dist;
                        count++;
                    }
                }
                else {
                    gcs().send_text(MAV_SEVERITY_INFO, "%x, %x", linebuf[NRA15_FRAME_LENGTH], linebuf[NRA15_FRAME_LENGTH+1]);
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm(), max_distance_cm() + NRA15_OUT_OF_RANGE_ADD_CM) * 0.01f;
        return true;
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_NRA15_ENABLED
