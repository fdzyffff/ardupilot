#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_ep4_in.h"

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

    FD1_msg_ep4_in& get_msg_ep4_in() { return _msg_ep4_in; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_ep4_in _msg_ep4_in;
};
