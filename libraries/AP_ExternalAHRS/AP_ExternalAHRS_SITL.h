#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SITL_ENABLED

#include "AP_ExternalAHRS_backend.h"


#include <GCS_MAVLink/GCS_MAVLink.h>
#include <SITL/SITL.h>


class AP_ExternalAHRS_SITL: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_SITL(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        build_packet_ins();
        build_packet_air();
    };

private:

    void update_thread();

    HAL_Semaphore sem;

    uint32_t last_ins_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_ahrs_pkt;

    AP_ExternalAHRS::ins_data_message_t imu_data;
    AP_ExternalAHRS::gps_data_message_t gps_data;
    AP_ExternalAHRS::baro_data_message_t baro_data;
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data;
    AP_ExternalAHRS::baro_alt_message_t bara_alt;

    void build_packet_ins();
    void build_packet_air();
    void handle_imu();
    void post_imu();
    void handle_gps();
    void post_gps();
    void handle_baro();
    void post_baro();
    void handle_airspeed();
    void post_airspeed();
    void handle_ahrs();

    class SITL::SIM *_sitl;

};

#endif  // AP_EXTERNAL_AHRS_SITL_ENABLED

