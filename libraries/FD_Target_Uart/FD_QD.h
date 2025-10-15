#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_msg_QD_S11.h"

class FD_QD {
public:

    FD_QD(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_QD(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_QD(const FD_QD &other) = delete;
    FD_QD &operator=(const FD_QD&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_msg_QD_S11& get_msg_QD_S11()   { return _msg_QD_S11; }
    // FD_msg_QD_S12& get_msg_QD_S12()   { return _msg_QD_S12; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_msg_QD_S11 _msg_QD_S11;
    // FD_msg_QD_S12 _msg_QD_S12;
};
