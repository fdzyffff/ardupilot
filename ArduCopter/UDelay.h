#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class UDelay {
public:
    static constexpr uint16_t BUFFER_SIZE = 200;

    UDelay();

    void init();
    void push(const Vector3f &value);
    bool get_idx(uint16_t step, Vector3f &value) const;

private:
    struct Sample {
        Vector3f value;
        uint32_t time_ms;
        bool valid;
    };

    Sample _buffer[BUFFER_SIZE];
    uint16_t _write_index;
    uint16_t _sample_count;
};
