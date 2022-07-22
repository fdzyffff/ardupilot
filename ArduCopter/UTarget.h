#pragma once

#include "UCam.h"
#include "UCapture.h"


class UTarget {

public:
    friend class UCam;
    friend class UCapture;

    // constructor, destructor
    UTarget();

    // initialise
    void init();

    bool is_active();
    int16_t cam_state();
    void mav_read();
    void do_cmd(float p1);

    uint8_t type() {return _type;}

    void handle_info(float p1, float p2, float p3, float p4);

    const Vector2f& get_raw_info() ;
    const Vector2f& get_correct_info() ;

    void update();

    float get_target_pitch_rate();
    float get_target_roll_angle();
    float get_target_yaw_rate();
    float get_current_angle_deg();

    bool display_info_new;
    float display_info_p1;
    float display_info_p2;
    float display_info_p3;
    float display_info_p4;
    int16_t display_info_count;
    UCam Ucam;
    UCapture Ucapture;
private:

    uint8_t _type;
};
