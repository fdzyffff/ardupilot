#pragma once

#include <HB1_UART/HB1_UART.h>
class UPayload {

public:

    // constructor, destructor
    UPayload();

    enum state_t {
        payload_none = 0,
        payload_parse = 1,
        payload_arm1 = 2,
        payload_arm2 = 3,
        payload_armfinal = 4,
        payload_fire = 5,
        payload_destroy = 6,
        payload_disarm = 99,
    };

    // initialise
    void init();
    void update();
    void set_state(state_t state);
    void send_state_msg(state_t state);
    void cmd_handle(int16_t cmd_in);
    bool initialised() {return _uart.initialized();}

private:

    FD_UART FD_uart_payload{AP_SerialManager::SerialProtocol_Payload};

    state_t _desire_state;
    state_t _current_state;
    uint32_t _last_state_ms;
    bool _new_msg;

    void msg_payload2apm_handle();
    void send_current_state_text();
    void do_next_state();
    void push_state();
    void flying_check();
};
