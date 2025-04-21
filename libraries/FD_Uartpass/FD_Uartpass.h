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
#include "FD_Uartpass_buffer.h"

class FD_Uartpass {
    friend class FD_Uartpass_buffer;
public:
    FD_Uartpass();
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void handle_msg(const mavlink_message_t &msg);
    void update();
    void read_uart() ;
    void send_mav();
    void set_target_sysid(uint16_t id_in);
    void push_byte(uint8_t temp);

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    AP_Int8 port_num;
    AP_Int32 port_baud;
    AP_Int8 info_print;
    AP_Int8 source_sys_id;

    FD_Uartpass_buffer data_buffer_instance;

    uint32_t last_check_ms;
};


using AP_HAL::millis;
