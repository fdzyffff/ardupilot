#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIDARM10P_ENABLED

#include "AP_Proximity_Backend_Serial.h"
#include "FD1_msg_M10P.h"

class AP_Proximity_LidarM10P : public AP_Proximity_Backend_Serial {
public:
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    void update() override;
    float distance_max_m() const override { return 6.5f; }
    float distance_min_m() const override { return 0.20f; }

private:
    static constexpr uint32_t TIMEOUT_MS = 300;
    static constexpr uint8_t FACE_COUNT = 8;

    struct FaceMinimum {
        bool valid;
        float angle_deg;
        float distance_m;
    };

    bool read_sensor_data();
    void process_frame();
    void reset_face_minima();
    void add_reading(float angle_deg, float distance_m);
    void publish_face_minima();

    FD1_msg_M10P _parser;
    uint32_t _last_distance_received_ms{0};
    FaceMinimum _face_minimum[FACE_COUNT]{};
};

#endif // AP_PROXIMITY_LIDARM10P_ENABLED
