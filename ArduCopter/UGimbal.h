#pragma once

#include <FD_UART/FD_UART.h>

class UGimbal {

public:

    // constructor, destructor
    UGimbal();

    void init();
    void update();
    void read_uart();
    void write_uart();
    void check_alive();
    void update_log();
    void update_target();
    // void send_cmd();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    struct {
        bool have_target;
        uint32_t target_ms;
        float cam_yaw;
        float cam_pitch;
    } status;

private:

    AP_HAL::UARTDriver* _port;

    uint32_t _last_update_ms;
    uint32_t _last_log_ms;
    bool _alive;

    // message structure
    FD_msg_QD_S11 uart_msg_QD_S11;
    FD_msg_QD_S12 uart_msg_QD_S12;
};
