#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_DYT_NEW_msg_DYTTELEM.h"
#include "FD_DYT_NEW_msg_DYTTARGET.h"
#include "FD_DYT_NEW_msg_APM2DYTCONTROL.h"
#include "FD_DYT_NEW_msg_APM2DYTTELEM.h"

class FD_DYT_NEW {
public:

    FD_DYT_NEW(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_DYT_NEW(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_DYT_NEW(const FD_DYT_NEW &other) = delete;
    FD_DYT_NEW &operator=(const FD_DYT_NEW&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_DYT_NEW_msg_DYTTELEM& get_msg_DYTTELEM()   { return _msg_DYTTELEM; }
    FD_DYT_NEW_msg_DYTTARGET& get_msg_DYTTARGET()   { return _msg_DYTTARGET; }
    FD_DYT_NEW_msg_APM2DYTCONTROL& get_msg_APM2DYTCONTROL()   { return _msg_APM2DYTCONTROL; }
    FD_DYT_NEW_msg_APM2DYTTELEM& get_msg_APM2DYTTELEM()   { return _msg_APM2DYTTELEM; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_DYT_NEW_msg_DYTTELEM _msg_DYTTELEM;
    FD_DYT_NEW_msg_DYTTARGET _msg_DYTTARGET;
    FD_DYT_NEW_msg_APM2DYTCONTROL _msg_APM2DYTCONTROL;
    FD_DYT_NEW_msg_APM2DYTTELEM _msg_APM2DYTTELEM;
};
