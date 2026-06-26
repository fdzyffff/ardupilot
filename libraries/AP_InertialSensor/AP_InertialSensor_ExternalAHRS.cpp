#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_ExternalAHRS.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_ExternalAHRS::AP_InertialSensor_ExternalAHRS(AP_InertialSensor &imu, uint8_t _serial_port) :
    AP_InertialSensor_Backend(imu),
    serial_port(_serial_port)
{
}

void AP_InertialSensor_ExternalAHRS::handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt)
{
    if (!started) {
        return;
    }
    _in_accel = pkt.accel;
    _in_gyro = pkt.gyro;
    _temperature = pkt.temperature;
}

bool AP_InertialSensor_ExternalAHRS::update(void)
{
    if (started) {
        update_accel(accel_instance);
        update_gyro(gyro_instance);
    }
    return started;
}

void AP_InertialSensor_ExternalAHRS::start()
{
    const float rate = AP::externalAHRS().get_IMU_rate();
    const bool reg_ok = _imu.register_gyro(gyro_instance, rate,
                           AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 1, DEVTYPE_SERIAL)) &&
                        _imu.register_accel(accel_instance, rate,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 2, DEVTYPE_SERIAL));

    if (!reg_ok) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ExternalAHRS::post_data, void), "EAHI", 1024, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        printf("Failed to allocate ExternalAHRS post_data thread\n");
        return;
    }

    _in_accel.z = -9.8;
    started = true;
}

void AP_InertialSensor_ExternalAHRS::accumulate()
{
    AP::externalAHRS().update();
}

// get a startup banner to output to the GCS
bool AP_InertialSensor_ExternalAHRS::get_output_banner(char* banner, uint8_t banner_len)
{
    const char* name = AP::externalAHRS().get_name();
    snprintf(banner, banner_len, "IMU%u: External: %s %0.0fHz",
             gyro_instance,
             (name != nullptr) ? name : "",
              AP::externalAHRS().get_IMU_rate());
    return true;
}

void AP_InertialSensor_ExternalAHRS::post_data()
{
    while (true) {
        if (started) {
            Vector3f rnd1 = rand_vec3f()*0.001f;
            Vector3f rnd2 = rand_vec3f()*0.0005f;
            _accel = _in_accel + rnd1;
            _gyro = _in_gyro + rnd2;
            _rotate_and_correct_accel(accel_instance, _accel);
            _notify_new_accel_raw_sample(accel_instance, _accel, AP_HAL::micros64());
            _publish_temperature(accel_instance, _temperature);
            _notify_new_gyro_sensor_rate_sample(gyro_instance, _gyro);
            _rotate_and_correct_gyro(gyro_instance, _gyro);
            _notify_new_gyro_raw_sample(gyro_instance, _gyro, AP_HAL::micros64());
        }
        hal.scheduler->delay_microseconds(1000);
    }
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

