#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_ue4_ahrs.h"
#include "FD1_msg_ID1.h"
#include "FD1_msg_ID2.h"
#include "FD1_msg_ID6.h"

class FD1_UART {
public:

    FD1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD1_UART(const FD1_UART &other) = delete;
    FD1_UART &operator=(const FD1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    uint32_t port_avaliable();

    FD1_msg_ue4_ahrs& get_msg_ue4_ahrs()   { return _msg_ue4_ahrs; }
    FD1_msg_ID1& get_msg_ID1()   { return _msg_ID1; }
    FD1_msg_ID2& get_msg_ID2()   { return _msg_ID2; }
    FD1_msg_ID6& get_msg_ID6()   { return _msg_ID6; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_ue4_ahrs _msg_ue4_ahrs;
    FD1_msg_ID1 _msg_ID1;
    FD1_msg_ID2 _msg_ID2;
    FD1_msg_ID6 _msg_ID6;
};
