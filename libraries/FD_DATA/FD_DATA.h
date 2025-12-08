#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>

struct PACKED FD_DATA_T {
    char serial_number[20];
    char uas_number[20];
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
    bool get_serial_number(char* serial_number);
    bool set_serial_number(char* serial_number);
    bool get_uas_number(char* uas_number);
    bool set_uas_number(char* uas_number);

    void update_flying_s();
    void update_allow_arm();
    void set_is_flying(bool in);
    bool get_runtime_flying(uint32_t& runtime_flying);
    bool reset_runtime_flying();

    void handle_message(const mavlink_message_t &msg);
    void send_mav_serial_number();
    void send_mav_uas_number();
    void send_mav_runtime_flying();

    void send_zfjl_sn(mavlink_channel_t chan);
    void send_zfjl_uas(mavlink_channel_t chan);
    void send_zfjl_gcs_heartbeat(mavlink_channel_t chan);

    bool pre_arm_checks(bool display_failure);

private:
    static FD_DATA *_singleton;

    static StorageAccess _storage;

    bool _is_flying;
    uint32_t _last_flying_ms;
    bool _allow_arm;
    uint32_t _last_gcs_heartbeat_ms;

    mavlink_zfjl_gcs_heartbeat_t zfjl_gcs_heartbeat_packet;

    FD_DATA_T local_data;

    AP_Int8 use_gcs_lock;
};


namespace AP {
    FD_DATA &fd_data();
};
