#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_MINS_ENABLED

#include "AP_ExternalAHRS_backend.h"


#include <GCS_MAVLink/GCS_MAVLink.h>
#include "FD1_msg_0XD1.h"
#include "FD1_msg_0XCC.h"
#include "FD1_msg_0XA1.h"
#include "FD1_msg_0XA2.h"


class AP_ExternalAHRS_MINS: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_MINS(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override {
        // build_packet_ins();
    };

    void send_ins_setting();
    void update_mag_cal();
    void send_mag_cal(uint8_t cmd);
    void handle_mag_cal();
    void update_airspeed();
    void post_gps();
    void post_imu();

    void update_log();
    void update_print();

protected:

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    void update_thread();

    AP_HAL::UARTDriver *uart_ins;
    HAL_Semaphore sem;

    uint32_t baudrate_ins;
    int8_t port_num_ins;
    bool port_open_ins = false;
    bool set_ins = false;

    uint32_t last_ahrs_pkt;
    uint32_t _last_cal_ms;
    bool mag_calibrating;

    struct {
        uint8_t cal_status;
        uint8_t attempt;
        uint8_t completion_pct;
        bool mag_calibrating;
        char msg[8];
        uint32_t last_cal_ms;
    } _mag_cal[2];

    void build_packet_ins();
    void print_ahrs_state();
    void handle_ahrs();

    FD1_msg_0XD1 _msg_0XD1;   //NAV系列惯导-惯导数据
    FD1_msg_0XCC _msg_0XCC;   //NAV系列惯导-磁校准指令
    FD1_msg_0XA1 _msg_0XA1;   //NAV系列惯导-罗盘校准状态
    FD1_msg_0XA2 _msg_0XA2;   //NAV系列惯导-光纤修正数据

};

#endif  // AP_EXTERNAL_AHRS_MINS_ENABLED

