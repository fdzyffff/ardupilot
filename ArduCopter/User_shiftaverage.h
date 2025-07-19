#pragma once
#include <AP_HAL/AP_HAL.h>

#define NUM_MY_DATAMAX 250

class User_shiftaverage {
public:
    User_shiftaverage() {}
    void init(float step_ms, uint8_t data_max);
    void push(float value);
    float get();

    bool _active;
    float _step_ms;
    uint8_t _idx;
    uint8_t _data_length;
    uint8_t _data_max;
    float _data[NUM_MY_DATAMAX];
    uint32_t _last_push_ms;
};

