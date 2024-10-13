#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_HXKY_1 : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(
        AP_InertialSensor &imu, 
        enum Rotation rotation);

    /* update accel and gyro state */
    // bool update() override;
    // void start() override;

    void start() override;
    bool update() override;

        // get a startup banner to output to the GCS
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    AP_InertialSensor_HXKY_1(AP_InertialSensor &imu,
                             enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> dev_accel;
    AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro;

    uint8_t gyro_instance;
    uint8_t accel_instance;
    enum Rotation rotation;

    bool init();

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool port_open = false;
    void update_thread();

    const uint8_t FRAME_HEAD = 0x36;  // 帧头  
    uint8_t IMU_state;  // 状态位  
    uint8_t decode_step = 0;  // 解帧步骤  
    uint16_t rx_checksum;  // 接收到的校验和  
    uint32_t calc_checksum;  // 计算的校验和  

    int16_t raw_gyro_X;
    int16_t raw_gyro_Y;
    int16_t raw_gyro_Z;

    int16_t raw_acc_X;
    int16_t raw_acc_Y;
    int16_t raw_acc_Z;

    int16_t roll_cd;  // 单位：百分之一度  
    int16_t pitch_cd;  // 单位：百分之一度  
    int16_t yaw_cd;  // 单位：百分之一度  

    int _init_step;  // 初始化状态  
};
