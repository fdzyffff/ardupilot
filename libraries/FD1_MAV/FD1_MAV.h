#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class FD1_MAV {
public:

    FD1_MAV(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD1_MAV(const FD1_MAV &other) = delete;
    FD1_MAV &operator=(const FD1_MAV&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    AP_HAL::UARTDriver* get_port();

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

};
