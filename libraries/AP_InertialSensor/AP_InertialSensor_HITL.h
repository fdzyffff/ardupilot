#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_HITL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_HITL(AP_InertialSensor &imu, uint8_t _bus_id);

    /* update accel and gyro state */
    bool update() override;
    void start() override;
    void accumulate() override;

    void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt) override;
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    void post_data();

    uint8_t bus_id;
    Vector3f _accel;
    Vector3f _gyro;
    float _temperature;
    bool started;
};
#endif // HAL_EXTERNAL_AHRS_ENABLED

