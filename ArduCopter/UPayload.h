#pragma once

#include <HB1_UART/HB1_UART.h>
class UPayload {

public:

    // constructor, destructor
    UPayload();

    enum state_t {
        payload_none = 0,
        payload_parse,
        payload_selfcheck,
        payload_voltup,
        payload_arm,
        payload_fire
    };

    // initialise
    void init();
    void update();
    void msg_payload2apm_handle();
    void set_state(state_t state);
    void cmd_handle(int16_t cmd_in);

private:

    HB1_UART _uart{AP_SerialManager::SerialProtocol_Payload};
    state_t _state;
    uint32_t _last_state_ms;
};
