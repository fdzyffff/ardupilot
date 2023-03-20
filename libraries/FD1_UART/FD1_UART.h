#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_gcs2nacelle.h"
#include "FD1_msg_nacelle2gcs.h"

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
    // void read();
    uint8_t read();
    void write();
    void set_port(AP_HAL::UARTDriver *port_in) {_port = port_in;}
    AP_HAL::UARTDriver* get_port() {return _port;}

    uint32_t port_avaliable();

    FD1_msg_gcs2nacelle& get_msg_gcs2nacelle()    { return _msg_gcs2nacelle; }
    FD1_msg_nacelle2gcs& get_msg_nacelle2gcs()    { return _msg_nacelle2gcs; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_gcs2nacelle _msg_gcs2nacelle;
    FD1_msg_nacelle2gcs _msg_nacelle2gcs;
};
