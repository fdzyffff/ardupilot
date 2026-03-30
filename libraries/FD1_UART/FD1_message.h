#pragma once
#include <AP_HAL/AP_HAL.h>

class FD1_message {
public:

    FD1_message() {_enable = true;}
    
    /* Do not allow copies */
    FD1_message(const FD1_message &other) = delete;
    FD1_message &operator=(const FD1_message&) = delete;

    virtual bool enable() {return _enable;}
    virtual void set_enable() {_enable = true;}
    virtual void set_disable() {_enable = false;}

    virtual void process_message(void) = 0;
    virtual void parse(uint8_t temp) = 0;
    virtual void swap_message() = 0;

    virtual void sum_check() = 0;

    float swap_message_float(float a1);
    int32_t swap_message_int32_t(int32_t a1);
    uint32_t swap_message_uint32_t(uint32_t a1);
    int16_t swap_message_int16_t(int16_t a1);
    uint16_t swap_message_uint16_t(uint16_t a1);

    void fill_int16_t(uint8_t* data, int16_t a1);
    void fill_uint16_t(uint8_t* data, uint16_t a1);
    void fill_int32_t(uint8_t* data, int32_t a1);
    void fill_uint32_t(uint8_t* data, uint32_t a1);
    void fill_float(uint8_t* data, float a1);
    bool _enable;
};
