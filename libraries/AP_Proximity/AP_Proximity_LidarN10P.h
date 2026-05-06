#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIDARN10P_ENABLED

#include "AP_Proximity_Backend_Serial.h"
#include "FD1_msg_N10P.h"

#define PROXIMITY_N10P_TIMEOUT_MS            300                               // requests timeout after 0.3 seconds

class AP_Proximity_LidarN10P : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    AP_Proximity_LidarN10P(AP_Proximity &_frontend,
                                 AP_Proximity::Proximity_State &_state,
                                 AP_Proximity_Params &_params,
                                 uint8_t serial_instance);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    void push_to_ring();
    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(int16_t angle_deg, uint16_t distance_mm);

    FD1_msg_N10P _msg_N10P;   //NAV系列惯导-惯导数据

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor

    class Local_face {
    public:
        void push_to_ring(float dist, float current_angle, float peak);

        AP_Proximity_LidarN10P *_frontend;
        float _angle_min;
        float _angle_max;
        float _last_min_dist;
        uint32_t _last_min_ms;
        uint32_t _last_pushed_ms;
        float _face_angle;
        bool _dist_pushed;
    };

    Local_face local_face[8];
};

#endif // AP_PROXIMITY_LIDARN10P_ENABLED
