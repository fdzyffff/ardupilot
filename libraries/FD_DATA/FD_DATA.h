#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>

struct PACKED FD_DATA_T {
    uint32_t serial_number;
    uint32_t runtime_flying;
};

class FD_DATA
{

public:
    FD_DATA();

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    FD_DATA(const FD_DATA &other) = delete;
    FD_DATA &operator=(const FD_DATA&) = delete;

    static FD_DATA *get_singleton() {
        return _singleton;
    }
    void update();
    bool get_serial_number(uint32_t& serial_number);
    bool set_serial_number(uint32_t serial_number);

    void update_flying_s();
    void set_is_flying(bool in);
    bool get_runtime_flying(uint32_t& runtime_flying);
    bool reset_runtime_flying();

    void handle_message(const mavlink_message_t &msg);
    void send_mav_serial_number(uint32_t serial_number);
    void send_mav_serial_number_get();
    void send_mav_runtime_flying(uint32_t runtime_flying);

private:
    static FD_DATA *_singleton;

    static StorageAccess _storage;

    bool _is_flying;
    uint32_t _last_flying_ms;

    // AP_Int8   test_mode;
};


namespace AP {
    FD_DATA &fd_data();
};
