#include "FD_DATA.h"

bool FD_DATA::pre_arm_checks() const
{
    if (use_gcs_lock.get() == 0) {
        return true;
    }
    return _last_gcs_heartbeat_ms != 0 &&
           AP_HAL::millis() - _last_gcs_heartbeat_ms <= 10000U;
}

void FD_DATA::set_mot_fail(bool enabled)
{
    const int8_t motor_number = mot_fail_number.get();
    if (motor_number >= 1 && motor_number <= 8) {
        _mot_fail = enabled;
        gcs().send_text(MAV_SEVERITY_INFO, "Mot fail %d %s",
                        motor_number, enabled ? "active" : "inactive");
    } else {
        _mot_fail = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Set ZFJL_MOT_FNUM to 1~8");
    }
}

bool FD_DATA::get_mot_fail(uint8_t mot_id) const
{
    const int8_t motor_number = mot_fail_number.get();
    return _mot_fail && motor_number >= 1 &&
           uint8_t(motor_number) == mot_id + 1U;
}

void FD_DATA::set_is_flying(bool is_flying)
{
    _is_flying = is_flying;
}

void FD_DATA::set_uav_status(uint8_t status)
{
    zfjl_uav_heartbeat_packet.status = status;
}

void FD_DATA::update_flying_s()
{
    const uint32_t now = AP_HAL::millis();
    if (_is_flying) {
        const uint32_t dt_ms = now - _last_flying_ms;
        _pending_flying_s += dt_ms / 1000U;
        _last_flying_ms = now - dt_ms % 1000U;
    } else {
        _last_flying_ms = now;
    }

    if (_pending_flying_s != 0 && set_flying_s(_pending_flying_s)) {
        _pending_flying_s = 0;
    }
}
