#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA15_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder.h"

class AP_RangeFinder_NRA15 : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_NRA15(_state, _params);
    }

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    float model_dist_max_cm() {return 10000;}
    bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[16];
    uint8_t linebuf_len;
};

#endif  // AP_RANGEFINDER_NRA15_ENABLED
