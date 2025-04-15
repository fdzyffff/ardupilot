#pragma once
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define NUM_MY_DATA 1
#define NUM_MY_DATALEN 200

class FD_Uartpass_buffer {
public:
    FD_Uartpass_buffer() {}
    void set_active();
    void update() ;
    uint16_t get_data(uint8_t (&data)[NUM_MY_DATALEN]);
    void push(uint8_t c);

    bool _active;
    uint32_t _last_active_ms;
    uint16_t data_idx;
    uint8_t _data[NUM_MY_DATALEN];
    uint8_t _id;
};

using AP_HAL::millis;