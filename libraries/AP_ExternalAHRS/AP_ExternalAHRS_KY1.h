#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_KY1_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "FD1_msg_KY1.h"

class AP_ExternalAHRS_KY1 : public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_KY1(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

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

    // post IMU data to InertialSensor
    void post_imu();

protected:

    uint8_t num_gps_sensors(void) const override {
        return 0;
    }

private:
    void update_thread();
    void process_packet();

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool port_open;

    uint32_t last_pkt_ms;

    // Frame parsing
    FD1_msg_KY1 _msg_KY1;   //开阳-1惯导数据
};

#endif  // AP_EXTERNAL_AHRS_KY1_ENABLED
