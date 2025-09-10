#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ESIM_ENABLED

#include "AP_ExternalAHRS_backend.h"


#include <GCS_MAVLink/GCS_MAVLink.h>
#include <SITL/SITL.h>


class AP_ExternalAHRS_ESIM: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_ESIM(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

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
        // build_packet_ins();
    };

private:

    void update_thread();

    AP_HAL::UARTDriver *uart_ins;
    uint32_t baudrate_ins;
    int8_t port_num_ins;
    bool port_open_ins = false;

    HAL_Semaphore sem;

    uint32_t last_ins_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_ahrs_pkt;

    AP_ExternalAHRS::ins_data_message_t imu_data;
    AP_ExternalAHRS::gps_data_message_t gps_data;
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data;

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    mavlink_sim_state_t sim_state_packet;

    void build_packet_ins();
    void handle_imu();
    void post_imu();
    void handle_gps();
    void post_gps();
    void handle_airspeed();
    void post_airspeed();
    void handle_ahrs();
    void send_packet_servo();
    void send_mav_message(mavlink_message_t *msg);
    class SITL::SIM *_sitl;

};

#endif  // AP_EXTERNAL_AHRS_ESIM_ENABLED

