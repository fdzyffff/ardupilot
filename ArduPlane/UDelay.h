#pragma once

#include <AP_HAL/AP_HAL.h>

#define UDELAY_BUFFER 100
class UDelay {
public:
    UDelay();
    
    void init();
    void push();
    bool get_idx(uint16_t step, float &roll, float &pitch, float &yaw);

private:
    struct {
        float roll;
        float pitch;
        float yaw;
        uint32_t time_ms;
    } _buffer[UDELAY_BUFFER];
    uint16_t _idx;
};
