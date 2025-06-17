#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_BIMCMD.h"
#include "FD1_msg_BIMSTATUS.h"

class FD1_UART {
public:

    FD1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD1_UART(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD1_UART(const FD1_UART &other) = delete;
    FD1_UART &operator=(const FD1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    uint8_t read_byte(void);
    void parse(uint8_t temp);
    void write();
    void write_byte(uint8_t temp);
    AP_HAL::UARTDriver* get_port() {return _port;}

    uint32_t port_avaliable();

    FD1_msg_BIMCMD& get_msg_BIMCMD()   { return _msg_BIMCMD; }
    FD1_msg_BIMSTATUS& get_msg_BIMSTATUS()   { return _msg_BIMSTATUS; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_BIMCMD _msg_BIMCMD;
    FD1_msg_BIMSTATUS _msg_BIMSTATUS;
};
