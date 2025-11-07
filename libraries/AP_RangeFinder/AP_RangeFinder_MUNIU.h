#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MUNIU_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder.h"
#include "FD1_UART/FD1_msg_RANGER.h"

class AP_RangeFinder_MUNIU : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return new AP_RangeFinder_MUNIU(_state, _params);
    }


protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[10];
    uint8_t linebuf_len;

    FD1_msg_RANGER _msg_ranger;
};

#endif  // AP_RANGEFINDER_MUNIU_ENABLED
