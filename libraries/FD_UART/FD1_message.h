#pragma once
#include <AP_HAL/AP_HAL.h>

class FD1_message {
public:

    FD1_message() {_enable = false;}
    
    /* Do not allow copies */
    FD1_message(const FD1_message &other) = delete;
    FD1_message &operator=(const FD1_message&) = delete;

    struct PACKED FD1_msg_header {
        uint8_t head;
        uint8_t source;
        uint8_t destination;
    };

    union PACKED Time_t {
        struct PACKED FD1_msg_time {
            uint32_t ms:10;
            uint32_t sec:6;
            uint32_t min:6;
            uint32_t h:5;
            uint32_t day:5;
        } bits;
        uint32_t v;
    };

    struct PACKED Target_t {
        int16_t yaw;
        int16_t pitch;
        int16_t x;
        int16_t y;
    };

    struct PACKED Loc_t {
        uint8_t quadrant;
        uint32_t lng;
        uint32_t lat;
        uint16_t abs_alt;
    };

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

    float get_angle(int16_t angle_in);
    int16_t pack_angle(float angle_in);

    virtual float swap_message_float(float a1);
    virtual int32_t swap_message_int32_t(int32_t a1);
    virtual uint32_t swap_message_uint32_t(uint32_t a1);
    virtual int16_t swap_message_int16_t(int16_t a1);
    virtual uint16_t swap_message_uint16_t(uint16_t a1);
    bool _enable;


    const float SF_ANGLE = 180.f/32768.0f;
};
