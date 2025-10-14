#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_TZ605_ENABLED

#include "AP_ExternalAHRS_backend.h"


#include <GCS_MAVLink/GCS_MAVLink.h>
#include <FD1_UART/FD1_UART.h>


class AP_ExternalAHRS_TZ605: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_TZ605(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

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
        if (port_open_ins) {build_packet_ins();}
        if (port_open_air) {build_packet_air();}
    };

private:

    void update_thread();

    AP_HAL::UARTDriver *uart_ins;
    AP_HAL::UARTDriver *uart_air;
    HAL_Semaphore sem;

    uint32_t baudrate_ins;
    int8_t port_num_ins;
    bool port_open_ins = false;
    uint32_t baudrate_air;
    int8_t port_num_air;
    bool port_open_air = false;

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
    void AP_ExternalAHRS_TZ605::get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out)

    FD1_msg_INS _msg_ins;   //.自定义消息解析器，处理外部传感器的专有协议
    FD1_msg_AIR _msg_air;

};

#endif  // AP_EXTERNAL_AHRS_TZ605_ENABLED

