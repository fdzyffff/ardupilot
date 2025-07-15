#pragma once

#include <FD_UART/FD_UART.h>

class UPayload {
public:

    // constructor, destructor
    UPayload();

    enum state_t {
        payload_none = 0,
        payload_parse,
        payload_arm1,
        payload_arm2,
        payload_armfinal,
        payload_fire,
        payload_destroy,
        payload_disarm,
    };

    // initialise
    void init();
    void update();
    void set_state(state_t state);
    void send_state_msg(state_t state);
    void cmd_handle(int16_t cmd_in);
    bool initialised() {return FD_uart_payload.initialized();}

private:

    FD_UART FD_uart_payload{AP_SerialManager::SerialProtocol_Payload};

    state_t _desire_state;
    state_t _current_state;
    uint32_t _last_state_ms;

    void msg_payload2apm_handle();
    void send_current_state_text();
    void push_state();
    // void flying_check();
};
