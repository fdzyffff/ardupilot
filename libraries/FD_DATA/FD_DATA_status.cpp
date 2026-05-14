#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"


bool FD_DATA::pre_arm_checks(bool display_failure)
{
    return _allow_arm;
}

void FD_DATA::update_allow_arm()
{
    if (use_gcs_lock.get() == 0) {
        _allow_arm = true;
        return;
    }

    if (AP_HAL::millis() - _last_gcs_heartbeat_ms > 10000) {
        _allow_arm = false;
    } else {
        _allow_arm = true;
    }
}

void FD_DATA::set_mot_fail(bool in)
{
    if (mot_fail_number.get() > 0) {
        _mot_fail = in;
        if (_mot_fail) {
            gcs().send_text(MAV_SEVERITY_INFO, "Mot fail %d active", mot_fail_number.get());
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Mot fail %d deactive", mot_fail_number.get());
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Set ZFJL_MOT_FNUM to 1~8");
    }
}

bool FD_DATA::get_mot_fail(uint8_t mot_id)
{
    if (_mot_fail) {
        // if (mot_fail_number.get() %2 == 0) {
        //     if ((mot_id+1) == mot_fail_number.get()) {
        //         return true;
        //     }
        //     if ((mot_id+1) == (mot_fail_number.get()-1)) {
        //         return true;
        //     }
        // } else {
        //     if ((mot_id+1) == mot_fail_number.get()) {
        //         return true;
        //     }
        //     if ((mot_id+1) == (mot_fail_number.get()+1)) {
        //         return true;
        //     }
        // }

        if ((mot_id+1) == mot_fail_number.get()) {
            return true;
        }
    }
    return false;
}

void FD_DATA::set_is_flying(bool in)
{
    _is_flying = in;
}

void FD_DATA::set_uav_status(uint8_t status_in)
{
    zfjl_uav_heartbeat_packet.status = status_in;
}

void FD_DATA::update_flying_s()
{
    static uint32_t dt_s = 0;
    if (_is_flying) {
        uint32_t dt_ms = AP_HAL::millis() - _last_flying_ms;
        dt_s += dt_ms/1000;
        _last_flying_ms = AP_HAL::millis() - dt_ms%1000;
    } else {
        _last_flying_ms = AP_HAL::millis();
    }

    if (set_flying_s(dt_s)) {
        dt_s = 0;
    }
}

void FD_DATA::update_sn_uas_check()
{
    if (AP_HAL::millis() - last_check_sn_uas_ms < 10000) {
        return;
    }

    last_check_sn_uas_ms = AP_HAL::millis();

    if (!sn_updated) {
        sn_updated = read_serial_number();
    }
    if (!uas_updated) {
        uas_updated = read_uas_number();
    }
}
