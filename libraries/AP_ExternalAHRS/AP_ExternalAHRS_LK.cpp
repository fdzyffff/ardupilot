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
  ExternalAHRS backend driven by the ground control station.

  There is no serial device and no separate thread: the ground station sends
  the interceptor position/velocity (CHASE protocol command 31016), UChase
  hands it to this backend, and the backend passes it to AP_GPS as external
  GPS data. The backend exists so that AP_GPS can create the
  AP_GPS_ExternalAHRS driver (GPS_TYPE=21), which is otherwise impossible
  with no external AHRS backend compiled in.
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_LK_ENABLED

#include "AP_ExternalAHRS_LK.h"
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LK::AP_ExternalAHRS_LK(AP_ExternalAHRS *_frontend,
                                       AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    // HUOJIAN: 只声明 GPS，避免被当成姿态/位置主源
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS));

    gcs().send_text(MAV_SEVERITY_INFO, "LK ExternalAHRS initialised");
}

// HUOJIAN: 无串口设备，返回 0 让 AP_GPS 创建 ExternalAHRS 驱动，数据由 31016 注入
int8_t AP_ExternalAHRS_LK::get_port(void) const
{
    return 0;
}

void AP_ExternalAHRS_LK::update()
{
    // HUOJIAN: 数据由 31016 驱动，这里只做无数据提示
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_status_ms >= 10000) {
        last_status_ms = now_ms;
        if (gps_count == 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "LK: no GPS data from GCS");
        }
    }
}

// HUOJIAN: 地面站注入的 GPS 数据转交 AP_GPS（实例 0）
bool AP_ExternalAHRS_LK::handle_ext_gps(const AP_ExternalAHRS::gps_data_message_t &pkt)
{
    last_gps_ms = AP_HAL::millis();
    gps_count++;

    AP::gps().handle_external(pkt, 0);
    return true;
}

bool AP_ExternalAHRS_LK::healthy(void) const
{
    // HUOJIAN: 2s 内收到过注入数据算健康
    return last_gps_ms != 0 && (AP_HAL::millis() - last_gps_ms) < 2000;
}

bool AP_ExternalAHRS_LK::initialised(void) const
{
    return last_gps_ms != 0;
}

bool AP_ExternalAHRS_LK::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // HUOJIAN: 不拦解锁，定位有效性由 AP_GPS/EKF 自身判定
    return true;
}

void AP_ExternalAHRS_LK::get_filter_status(nav_filter_status &status) const
{
    status = {};
    if (last_gps_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - last_gps_ms < 2000) {
        status.flags.horiz_pos_abs = 1;
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_vel = 1;
        status.flags.vert_pos = 1;
        status.flags.vert_vel = 1;
        status.flags.using_gps = 1;
    }
}

bool AP_ExternalAHRS_LK::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return false;
}

#endif  // AP_EXTERNAL_AHRS_LK_ENABLED
