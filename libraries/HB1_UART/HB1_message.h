#pragma once
#include <AP_HAL/AP_HAL.h>

class HB1_message {
public:

    HB1_message() {_enable = false;}
    
    /* Do not allow copies */
    HB1_message(const HB1_message &other) = delete;
    HB1_message &operator=(const HB1_message&) = delete;

    virtual bool enable() {return _enable;}
    virtual void set_enable() {_enable = true;}
    virtual void set_disable() {_enable = false;}

    virtual void process_message(void) = 0;
    virtual void parse(uint8_t temp) = 0;
    virtual void swap_message() = 0;
    virtual void cal_sumcheck() = 0;

    void swap_message_sub(uint8_t &p1, uint8_t &p2) ;

    void swap_message_sub(uint8_t &p1, uint8_t &p2, uint8_t &p3, uint8_t &p4) {
        swap_message_sub(p1, p4);
        swap_message_sub(p2, p3);
    }

    bool _enable;
};
