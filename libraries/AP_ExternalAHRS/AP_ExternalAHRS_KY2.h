#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_KY2_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "FD1_msg_KY2.h"

class AP_ExternalAHRS_KY2 : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_KY2(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data (no-op, handled by thread)
    void update() override {};

    // post IMU and GPS data
    void post_imu();
    void post_gps();

    // logging
    void update_log();

    // config command sending
    void send_config();

    static const struct AP_Param::GroupInfo var_info[];

protected:
    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    void update_thread();
    void handle_ahrs();
    void print_ahrs_state();

    // GPS fix type mapping
    AP_GPS_FixType ky2_fix_to_ap(uint8_t ky2_fix);

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool port_open;

    uint32_t last_pkt_ms;
    uint32_t last_gps_post_ms;
    uint32_t last_log_ms;
    uint32_t _last_location_ms;
    bool _has_new_frame;

    // Parameters
    AP_Int8 _log_level;    // 0=off, 1=raw every frame, 2=100ms filtered
    AP_Int8 _cfg_mode;     // 0=off, 1=send once reset to 0, 2=send and keep

    // Frame parsing
    FD1_msg_KY2 _msg_KY2;

    bool _config_sent;
};

#endif  // AP_EXTERNAL_AHRS_KY2_ENABLED
