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
  ExternalAHRS backend which has no serial device: the GPS data is injected
  by the ground control station (CHASE protocol, command 31016) and handed to
  AP_GPS as if it came from an external AHRS unit.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_LK_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_LK : public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_LK(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override { return "LK"; }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;
    uint8_t num_gps_sensors(void) const override { return 1; }

    // HUOJIAN: 地面站注入的 GPS 数据入口（31016 拦截机信息）
    bool handle_ext_gps(const AP_ExternalAHRS::gps_data_message_t &pkt) override;

private:

    uint32_t last_gps_ms;
    uint32_t gps_count;
    uint32_t last_status_ms;
};

#endif  // AP_EXTERNAL_AHRS_LK_ENABLED
