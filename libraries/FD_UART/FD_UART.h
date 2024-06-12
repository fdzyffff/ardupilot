#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

// #include "FD_msg_ue4_ahrs.h"

class FD_UART {
public:

    FD_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD_UART(const FD_UART &other) = delete;
    FD_UART &operator=(const FD_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();
    AP_HAL::UARTDriver* get_port() {return _port;}

    uint32_t port_avaliable();

    // FD_msg_ue4_ahrs& get_msg_ue4_ahrs()   { return _msg_ue4_ahrs; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    // FD_msg_ue4_ahrs _msg_ue4_ahrs;
};
