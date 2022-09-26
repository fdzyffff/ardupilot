#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_ep4_in.h"
#include "FD1_msg_ep4_out.h"
#include "FD1_msg_ts.h"

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

    FD1_msg_ep4_in& get_msg_ep4_in()   { return _msg_ep4_in; }
    FD1_msg_ep4_out& get_msg_ep4_out() { return _msg_ep4_out; }

    FD1_msg_ts& get_msg_ts_in()    { return _msg_ts_in; }
    FD1_msg_ts& get_msg_ts_out()   { return _msg_ts_out; }
    FD1_msg_ts& get_msg_ts_route() { return _msg_ts_route; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_ep4_in _msg_ep4_in;
    FD1_msg_ep4_out _msg_ep4_out;
    FD1_msg_ts _msg_ts_in;
    FD1_msg_ts _msg_ts_out;
    FD1_msg_ts _msg_ts_route;
};
