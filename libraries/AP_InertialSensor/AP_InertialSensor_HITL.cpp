#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_HITL.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_HITL::AP_InertialSensor_HITL(AP_InertialSensor &imu, uint8_t _bus_id) :
    AP_InertialSensor_Backend(imu),
    bus_id(_bus_id)
{
}

void AP_InertialSensor_HITL::handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt)
{
    if (!started) {
        return;
    }
    Vector3f accel = pkt.accel;
    Vector3f gyro = pkt.gyro;

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, AP_HAL::micros64());

    _publish_temperature(accel_instance, pkt.temperature);

    _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, AP_HAL::micros64());
}

bool AP_InertialSensor_HITL::update(void)
{
    if (started) {
        update_accel(accel_instance);
        update_gyro(gyro_instance);
    }
    return started;
}

void AP_InertialSensor_HITL::start()
{
    const float rate = AP::externalAHRS().get_IMU_rate();
    if (_imu.register_gyro(gyro_instance, rate, 
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 1, DEVTYPE_SITL)) &&
        _imu.register_accel(accel_instance, rate,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 2, DEVTYPE_SITL))) {
        started = true;
    }
}

void AP_InertialSensor_HITL::accumulate()
{
    // AP::externalAHRS().update();
}

// get a startup banner to output to the GCS
bool AP_InertialSensor_HITL::get_output_banner(char* banner, uint8_t banner_len)
{
    const char* name = AP::externalAHRS().get_name();
    snprintf(banner, banner_len, "IMU%u: External: %s %0.0fHz",
             gyro_instance,
             (name != nullptr) ? name : "",
              AP::externalAHRS().get_IMU_rate());
    return true;
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

