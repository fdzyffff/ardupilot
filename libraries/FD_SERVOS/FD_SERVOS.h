#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

#include "FD_msg_SERVO_receive.h"
#include "FD_msg_SERVO_15.h"
#include "FD_msg_SERVO_17.h"
#include "FD_msg_SERVO_23.h"
#include "FD_msg_SERVO_24.h"

#define FD_SERVO_MAX_NUM 6

class FD_SERVO;

class FD_SERVOS {
public:

    FD_SERVOS();

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    FD_SERVOS(const FD_SERVOS &other) = delete;
    FD_SERVOS &operator=(const FD_SERVOS&) = delete;

    static FD_SERVOS *get_singleton() {
        return _singleton;
    }


    // init - perform required initialisation
    bool initialized() {return _initialized;}
    AP_HAL::UARTDriver *get_port() {return _port;}

    bool init();
    void update();
    void read_uart(void);
    void update_control();
    void set_speed(float speed_norm_in, float turn_norm_in);
    void set_enable(bool enable_in);

    FD_SERVO* servo_instance[FD_SERVO_MAX_NUM];

    FD_msg_SERVO_receive& get_msg_SERVO_receive()   { return _msg_SERVO_receive; }
private:
    static FD_SERVOS *_singleton;

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;
    bool _enable;

    AP_Float servo_vel;

    // 状态回复
    FD_msg_SERVO_receive _msg_SERVO_receive;
};


class FD_SERVO {
public:

    FD_SERVO(FD_SERVOS *fronted, uint8_t id_in):
    _fronted(fronted),
    _id(id_in)
    {
        ;
    }

    void update();
    void set_vel(float servo_vel_in);
    void set_value(float value_in);
    void do_stop();
    void do_reset();
    void do_speed();

private:
    FD_SERVOS *_fronted;
    uint8_t _id;

    uint32_t last_update_ms;
    float value;
    uint32_t last_reset_ms;
    float _servo_vel;
    uint8_t stop_count;

    // 设置ID等的写入指令
    // FD_msg_SERVO_4  _msg_SERVO_4;
    // 多圈控制指令
    FD_msg_SERVO_15 _msg_SERVO_15;
    // 多圈重置指令
    FD_msg_SERVO_17 _msg_SERVO_17;
    // 状态查询
    // FD_msg_SERVO_22 _msg_SERVO_22;
    // 重置零点指令
    // FD_msg_SERVO_23 _msg_SERVO_23;
    // 停止指令
    FD_msg_SERVO_24 _msg_SERVO_24;

    FD_msg_SERVO_15& get_msg_SERVO_15()   { return _msg_SERVO_15; }
};

namespace AP {
    FD_SERVOS &fd_servos();
};
