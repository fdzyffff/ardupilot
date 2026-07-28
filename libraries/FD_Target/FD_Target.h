#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class FD_Target_Base {
public:
    FD_Target_Base();
    virtual ~FD_Target_Base() {}

    virtual bool init() { return false; }
    virtual void update() = 0;
    virtual void handle_msg(const mavlink_message_t &msg) {}

    bool is_valid() const { return _valid; }
    void handle_info(float p1, float p2);
    bool get_info(float &p1, float &p2);
    void recover_info();
    uint8_t get_type();
    void set_type(uint8_t type_in);
    void set_valid(bool valid_in);
    virtual Location &get_target_loc() { return _target_loc; }

protected:
    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
    uint8_t _type;
    Location _target_loc;
};
