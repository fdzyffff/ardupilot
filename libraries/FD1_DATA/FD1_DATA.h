#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class FD1_DATA
{

public:
    FD1_DATA();

    /* Do not allow copies */
    FD1_DATA(const FD1_DATA &other) = delete;
    FD1_DATA &operator=(const FD1_DATA&) = delete;

    static FD1_DATA *get_singleton() {
        return _singleton;
    }

    // yaw in degrees if available
    bool get_yaw_deg(float &yaw_deg, float &accuracy_deg);
    void set_yaw_deg(float yaw_deg, float accuracy_deg);

private:
    static FD1_DATA *_singleton;
    bool _new_data;
    float _yaw_deg;
    float _accuracy_deg;
};


namespace AP {
    FD1_DATA &fd1_data();
};
