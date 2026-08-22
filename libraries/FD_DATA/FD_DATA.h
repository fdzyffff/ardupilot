#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <StorageManager/StorageManager.h>

struct PACKED FD_DATA_T {
    char serial_number[20];
    char uas_number[20];
    uint32_t runtime_flying;
};

class FD_DATA
{
public:
    static constexpr uint8_t IDENTITY_FIELD_SIZE = 20;
    static constexpr uint8_t IDENTITY_TEXT_SIZE = IDENTITY_FIELD_SIZE + 1;

    FD_DATA();

    FD_DATA(const FD_DATA &other) = delete;
    FD_DATA &operator=(const FD_DATA &other) = delete;

    static FD_DATA *get_singleton()
    {
        return _singleton;
    }

    static const AP_Param::GroupInfo var_info[];

    void update();

    bool get_serial_number(char serial_number[IDENTITY_TEXT_SIZE]);
    bool set_serial_number(const char *serial_number, uint8_t length);
    bool get_uas_number(char uas_number[IDENTITY_TEXT_SIZE]);
    bool set_uas_number(const char *uas_number, uint8_t length);
    bool get_runtime_flying(uint32_t &runtime_flying);
    bool reset_runtime_flying();

    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);
    void send_zfjl_sn(mavlink_channel_t chan);
    void send_zfjl_rt(mavlink_channel_t chan);
    void send_zfjl_uas(mavlink_channel_t chan);
    void send_zfjl_uav_heartbeat(mavlink_channel_t chan);

    bool pre_arm_checks() const;

    void set_is_flying(bool is_flying);
    void set_uav_status(uint8_t status);

    void set_mot_fail(bool enabled);
    bool get_mot_fail(uint8_t mot_id) const;

    const mavlink_zfjl_gcs_heartbeat_t &get_gcs_heartbeat_msg() const
    {
        return zfjl_gcs_heartbeat_packet;
    }

private:
    static FD_DATA *_singleton;
    static StorageAccess _storage;

    bool load_record();
    bool commit_record(const FD_DATA_T &candidate);
    bool set_flying_s(uint32_t dt_s);
    void update_flying_s();

    void handle_message_sn(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_message_uas(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_message_rt(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_message_gcs_heartbeat(const mavlink_message_t &msg);


    uint32_t _last_update_flying_ms = 0;
    bool _is_flying = false;
    uint32_t _last_flying_ms = 0;
    uint32_t _pending_flying_s = 0;
    uint32_t _last_gcs_heartbeat_ms = 0;
    bool _mot_fail = false;

    mavlink_zfjl_gcs_heartbeat_t zfjl_gcs_heartbeat_packet{};
    mavlink_zfjl_uav_heartbeat_t zfjl_uav_heartbeat_packet{};
    FD_DATA_T local_data{};
    bool record_loaded = false;

    AP_Int8 use_gcs_lock;
    AP_Int8 uav_type;
    AP_Int8 mot_fail_number;
};

namespace AP {
FD_DATA &fd_data();
}
