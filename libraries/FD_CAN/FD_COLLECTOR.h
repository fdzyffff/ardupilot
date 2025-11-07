#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN_2.h"

class FD_CAN_2;

class FD_COLLECTOR {
public:
    friend class FD_CAN_2;

    FD_COLLECTOR(FD_CAN_2* frotend);
    ~FD_COLLECTOR();

    /* Do not allow copies */
    FD_COLLECTOR(const FD_COLLECTOR &other) = delete;
    FD_COLLECTOR &operator=(const FD_COLLECTOR&) = delete;

    void update_send();
    void send_cmd(uint32_t id, uint8_t *data);
    void send_utc();
    FD_CAN_2* _frotend_ptr;

    float left_wheel; // 左轮速 RPM
    float right_wheel; // 右轮速 RPM
    float left_brake_in; // 左刹车阀输入信号 mA
    float left_brake_back; // 左刹车阀反馈信号 V
    float right_brake_in; // 右刹车阀输入信号 mA
    float right_brake_back; // 右刹车阀反馈信号 V
    float alt; // 高度 cm
    float arspd_tas; // 真空速 m/s
    float climb_rate; // 升降速度 cm/s
    float aoa; // 迎角 degree
    float ssa; // 侧滑角 degree
    float roll; // 滚动角 degree
    float yaw; // 航向角 degree
    float rate_x; // X轴角速度 degree/s
    float rate_y; // Y轴角速度 degree/s
    float rate_z; // Z轴角速度 degree/s
    float acc_x; // X轴加速度 m/s/s
    float acc_y; // Y轴加速度 m/s/s
    float acc_z; // Z轴加速度 m/s/s

    union PACKED{
        float v;
        uint8_t data[4];
    } tmp_float_to_data;

    union PACKED{
        uint32_t v;
        uint8_t data[4];
    } tmp_uint32t_to_data;

    struct status_t {
        uint32_t last_ask_status_ms;
    };

    status_t status;
    uint8_t _data[8];
};
