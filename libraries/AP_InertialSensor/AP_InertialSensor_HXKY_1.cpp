#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_InertialSensor_HXKY_1.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

// const extern AP_HAL::HAL& hal;

extern const AP_HAL::HAL& hal;


AP_InertialSensor_HXKY_1::AP_InertialSensor_HXKY_1(AP_InertialSensor &imu,
                                                         enum Rotation _rotation):
    AP_InertialSensor_Backend(imu),
    rotation(_rotation)
{
    _init_step = 1;
}

AP_InertialSensor_Backend *
AP_InertialSensor_HXKY_1::probe(AP_InertialSensor &imu,
                                enum Rotation rotation)
{
    auto sensor = new AP_InertialSensor_HXKY_1(imu, rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_InertialSensor_HXKY_1::init()
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_HXKY_1, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_HXKY_1, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY no UART");
        return false;
    }

    _init_step = 2;

    return true;
}



void AP_InertialSensor_HXKY_1::start()
{
    _init_step = 3;
    if (!_imu.register_gyro(gyro_instance, 200,
                           AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, port_num, 1, DEVTYPE_SERIAL)) ||
        !_imu.register_accel(accel_instance, 200,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, port_num, 2, DEVTYPE_SERIAL))) {
        return;
    }

    // setup sensor rotations from probe()
    // 在 probe() 函数中设置传感器旋转。
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_HXKY_1::update_thread, void), "HXKY", 1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create HMS thread");
        _init_step = 9;
    }

    _init_step = 4;
}

// get a startup banner to output to the GCS

bool AP_InertialSensor_HXKY_1::get_output_banner(char *banner, uint8_t banner_len) 
{
    snprintf(banner, banner_len, "HXKY_1 IMU at port %u, state: %d", port_num, _init_step);

    return true;
}

void AP_InertialSensor_HXKY_1::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(115200, 128, 128);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HXKY_1 open UART");
    }

    while (true) {
        _init_step ++;
        
        uint32_t nbytes = uart->available();
        for (uint32_t i = 0; i < nbytes; i++) {
            uint8_t b = uart->read();
            switch (decode_step) {
                case 0:
                    if(b == FRAME_HEAD) {
                        decode_step ++;
                        calc_checksum = b;
                    }
                break;

                case 1:
                    raw_gyro_X = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 2:
                    raw_gyro_X += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 3:
                    raw_gyro_Y = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 4:
                    raw_gyro_Y += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 5:
                    raw_gyro_Z = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 6:
                    raw_gyro_Z += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 7:
                    raw_acc_X = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 8:
                    raw_acc_X += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 9:
                    raw_acc_Y = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 10:
                    raw_acc_Y += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 11:
                    raw_acc_Z = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 12:
                    raw_acc_Z += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 13:
                    roll_cd = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 14:
                    roll_cd += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 15:
                    pitch_cd = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 16:
                    pitch_cd += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 17:
                    yaw_cd = (int16_t)(b << 8);
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 18:
                    yaw_cd += b;
                    decode_step ++;
                    calc_checksum += b;
                break;

                case 19:
                    IMU_state = b;
                    decode_step ++;
                    calc_checksum += ((uint16_t)(b << 8));
                break;

                case 20:
                    rx_checksum = b;
                    decode_step ++;
                break;

                case 21:
                    decode_step = 0;
                    rx_checksum += ((uint16_t)(b << 8));
                    if(rx_checksum == (uint16_t)(calc_checksum & 0x0000FFFF)) {
                        Vector3f accel, gyro;
                        accel = Vector3f(raw_acc_X, raw_acc_Y, raw_acc_Z);
                        accel *= (0.000488f * GRAVITY_MSS);

                        gyro = Vector3f(raw_gyro_X, raw_gyro_Y, raw_gyro_Z);
                        gyro *= radians(0.0175f);

                        _rotate_and_correct_accel(accel_instance, accel);
                        _rotate_and_correct_gyro(gyro_instance, gyro);

                        _notify_new_accel_raw_sample(accel_instance, accel);
                        _notify_new_gyro_raw_sample(gyro_instance, gyro);
                    }
                    else
                    {
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HMS checksum failed");
                    }
                break;
            
            default:
                decode_step = 0;
                break;
            }
        }
        
        hal.scheduler->delay_microseconds(100);
    }
}

bool AP_InertialSensor_HXKY_1::update(void)
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
