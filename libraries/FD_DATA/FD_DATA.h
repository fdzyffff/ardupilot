#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

class FD_DATA
{

public:
    FD_DATA();

    /* Do not allow copies */
    FD_DATA(const FD_DATA &other) = delete;
    FD_DATA &operator=(const FD_DATA&) = delete;

    static FD_DATA *get_singleton() {
        return _singleton;
    }

    // void set_tof_matrix(uint8_t (&data)[NUM_TOFMATRIX_DATALEN]);
    void send_mav_tof_matrix(mavlink_wxbs_tof_distance_t *packet);
private:
    static FD_DATA *_singleton;

    static StorageAccess _storage;
};


namespace AP {
    FD_DATA &fd_data();
};
