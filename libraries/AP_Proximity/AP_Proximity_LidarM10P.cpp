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

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIDARM10P_ENABLED

#include "AP_Proximity_LidarM10P.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_LidarM10P::AP_Proximity_LidarM10P(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state,
                                                         AP_Proximity_Params &_params,
                                                         uint8_t serial_instance) :
    AP_Proximity_Backend_Serial(_frontend, _state, _params, serial_instance)
{
    for (uint8_t i_face = 0; i_face < 8; i_face++) {
        local_face[i_face]._frontend = this;
        local_face[i_face]._face_angle = i_face * 45;
        local_face[i_face]._angle_min = wrap_360((float)(i_face * 45) - 22.5f);
        local_face[i_face]._angle_max = wrap_360((float)(i_face * 45) + 22.5f);
        local_face[i_face]._last_min_dist = 0.0f;
        local_face[i_face]._dist_pushed =false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "M10 Plus Initialized");
}

// update the state of the sensor
void AP_Proximity_LidarM10P::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_M10P_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_LidarM10P::distance_max() const
{
    return 6.5f;
}
float AP_Proximity_LidarM10P::distance_min() const
{
    return 0.20f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_LidarM10P::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = _uart->available();

    while (nbytes-- > 0) {
        uint8_t temp = _uart->read();    //. 逐字节读取原始 uint8_t 数据
        _msg_M10P.parse(temp);           //. 传入解析器

        if (_msg_M10P._msg_1.updated) {
            push_to_ring();
            _msg_M10P._msg_1.updated = false;
        }
    }
    return (message_count > 0);
}

void AP_Proximity_LidarM10P::push_to_ring()
{
    float temp_start_angle = 0.01f * (float)(UINT16_VALUE(_msg_M10P._msg_1.content.data[4], _msg_M10P._msg_1.content.data[5]));
    float valid_point = 0;
    for (uint8_t i_dist = 0; i_dist < 70; i_dist++) {
        uint8_t i_dist_idx = 8 + 2 * i_dist;
        if (UINT16_VALUE(_msg_M10P._msg_1.content.data[i_dist_idx], _msg_M10P._msg_1.content.data[i_dist_idx + 1]) != 0XFF) {
            valid_point += 1.0f;
        }
    }

    valid_point = constrain_float(valid_point, 1.0f, 100.f);
    for (uint8_t i_dist = 0; i_dist < 70; i_dist++) {
        uint8_t i_dist_idx = 8 + 2 * i_dist;
        float temp_dist = (float)(UINT16_VALUE(_msg_M10P._msg_1.content.data[i_dist_idx], _msg_M10P._msg_1.content.data[i_dist_idx + 1]));

        float temp_current_angle = temp_start_angle + ((float)i_dist_idx/valid_point)*15.f;
        temp_current_angle = wrap_360(temp_current_angle);

        if (temp_dist < 1.0f) {
            break;
        }

        for (uint8_t i_face = 0; i_face < 8; i_face++) {
            local_face[i_face].push_to_ring(temp_dist, temp_current_angle);
        }
    }
}

// process reply
void AP_Proximity_LidarM10P::update_sector_data(int16_t angle_deg, uint16_t distance_mm)
{
    // Get location on 3-D boundary based on angle to the object
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
    if ((distance_mm != 0xffff) && !ignore_reading(angle_deg, distance_mm * 0.001f, false)) {
        frontend.boundary.set_face_attributes(face, angle_deg, ((float) distance_mm) / 1000, state.instance);
        // update OA database
        database_push(angle_deg, ((float) distance_mm) / 1000);
    } else {
        frontend.boundary.reset_face(face, state.instance);
    }
    _last_distance_received_ms = AP_HAL::millis();
}

void AP_Proximity_LidarM10P::Local_face::push_to_ring(float dist, float current_angle)
{
    // if (peak < 50.f) {return;}

    bool in_face = (wrap_180(current_angle - _angle_min) > 0.0f) && (wrap_180(_angle_max - current_angle) > 0.0f);
    // if (in_face) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "current_angle %0.1f, _angle_max %0.1f, _angle_min %0.1f", current_angle, _angle_max, _angle_min);
    // }
    if (in_face) {
        if (AP_HAL::millis() - _last_min_ms > 30) {
            _last_min_dist = -100.0f;
        }

        if (dist < _last_min_dist || _last_min_dist < 0.0f) {
            _last_min_dist = dist;
            _last_min_ms = AP_HAL::millis();
            _dist_pushed = false;
        }
    }

    if (!in_face && !_dist_pushed) {
        if (AP_HAL::millis() - _last_pushed_ms > 50) {
            if (_frontend != nullptr) {
                _frontend->update_sector_data(_face_angle, _last_min_dist); // meter

                // gcs().send_text(MAV_SEVERITY_INFO, "_face_angle %0.1f, _last_min_dist %0.1f, dist %0.1f", _face_angle, _last_min_dist*0.001f, dist*0.001f);
            }
            _last_pushed_ms = AP_HAL::millis();
            _dist_pushed = true;
        }
    }
}

#endif // AP_PROXIMITY_LIDARM10P_ENABLED
