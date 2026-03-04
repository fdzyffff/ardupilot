#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_HITL_ENABLED

#include "AP_ExternalAHRS_backend.h"

#include <GCS_MAVLink/GCS_MAVLink.h>


class AP_ExternalAHRS_HITL: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_HITL(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

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

    // check for new data
    void update() override {
        ;
    };

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:

    void update_thread(void);
    void build_packet_hitl();
    void handle_sensor(mavlink_hil_sensor_t &in_packet);
    void handle_hil_gps(mavlink_hil_gps_t &in_packet);
    void post_gps();
    void post_imu();
    void update_log();
    void update_print();
    void update_actuator_controls();
    void send_mavlink_message(mavlink_message_t *msg);

    HAL_Semaphore sem;

    AP_HAL::UARTDriver *uart_hitl;
    uint32_t baudrate_hitl;
    int8_t port_num_hitl;
    bool port_open_hitl = false;

    uint32_t last_ins_pkt;
    uint32_t last_gps_pkt;

    float ins_frame_count;
    uint32_t _last_ins_print;
    uint32_t _last_gps_print;
    uint32_t _last_gps_post_ms;
    uint32_t _last_log_ms;
    uint32_t _last_global_print;
    uint32_t _last_srv_post_ms;

    mavlink_hil_sensor_t hil_sensor_packet;
    mavlink_hil_gps_t hil_gps_packet;
    mavlink_hil_actuator_controls_t hil_actuator_controls_packet;

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;
};

#endif  // AP_EXTERNAL_AHRS_HITL_ENABLED

