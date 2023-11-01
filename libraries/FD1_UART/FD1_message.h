#pragma once
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class FD1_message {
public:

    FD1_message() {_enable = false;}
    
    /* Do not allow copies */
    FD1_message(const FD1_message &other) = delete;
    FD1_message &operator=(const FD1_message&) = delete;

    virtual bool enable() {return _enable;}
    virtual void set_enable() {_enable = true;}
    virtual void set_disable() {_enable = false;}

    virtual void process_message(void) = 0;
    virtual void parse(uint8_t temp) = 0;
    virtual void swap_message() = 0;

    void swap_message_sub(uint8_t &p1, uint8_t &p2) ;

    void swap_message_sub(uint8_t &p1, uint8_t &p2, uint8_t &p3, uint8_t &p4) {
        swap_message_sub(p1, p4);
        swap_message_sub(p2, p3);
    }

    virtual float swap_message_float(float a1);
    virtual int32_t swap_message_int32_t(int32_t a1);
    virtual uint32_t swap_message_uint32_t(uint32_t a1);
    virtual int16_t swap_message_int16_t(int16_t a1);
    virtual uint16_t swap_message_uint16_t(uint16_t a1);
    bool _enable;
};
