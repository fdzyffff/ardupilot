#pragma once

#include <FD1_UART/FD1_UART.h>

class UK230 {

public:

    // constructor, destructor
    UK230();

    // initialise
    void init();

    void handle_message(const mavlink_message_t msg);

    void update();

private:

    // LowPassFilterVector3f _filter_target_cm;

    // Vector3f _raw_target_cm;

    Vector3f bf_info;
    Vector3f efb_info;
    float _target_pitch_rate;
    float _target_roll_rate;
    float _target_yaw_rate;
    float _target_dist_cm;
    float _target_bf_vel_x;
    float _target_bf_vel_y;
    uint32_t _last_ms;
    bool _valid;

    LowPassFilterVector3f efb_info_filt;

};
