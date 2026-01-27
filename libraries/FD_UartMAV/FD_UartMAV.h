#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

class FD_UartMAV {
public:
    FD_UartMAV();
    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    FD_UartMAV(const FD_UartMAV &other) = delete;
    FD_UartMAV &operator=(const FD_UartMAV&) = delete;

    static FD_UartMAV *get_singleton() {
        return _singleton;
    }

    void init();
    bool initialized();
    void handle_msg(const mavlink_message_t &msg);
    void update();
    void read_uart() ;
    void send_mav();
    void send_raw_gps();
    void set_target_sysid(uint16_t id_in);
    void send_msg(mavlink_message_t *msg);
    void push_byte(uint8_t temp);

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    AP_Int8 port_num;
    AP_Int8 info_print;
    AP_Int8 sys_id;
    AP_Float filt_gyro_hz;
    AP_Float filt_acc_hz;
    AP_Int8 use_external;

    LowPassFilterVector3f _imu_gyro;
    LowPassFilterVector3f _imu_acc;

    uint32_t last_check_ms;

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

private:
    
    static FD_UartMAV *_singleton;
};


using AP_HAL::millis;


namespace AP {
    FD_UartMAV &fd_uartmav();
};
