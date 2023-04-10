#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class UBase {

public:

    // constructor, destructor
    UBase();

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p14;
        uint16_t count;
    } display_info;

    enum class throttle_pos {
        LOW = 0,
        MID = 1,
        HIGH = 2,
    };

    enum class land_stage {
        HOLDOFF = 0;
        DESCEND = 1;
        APPROACH = 2;
        IDLE = 3;
    };

    struct mlstate_t {
        throttle_pos throttle_switch;
        land_stage landing_stage;
        Location current_pos;
        Location target_pos;
        Vector3f target_velocity;
        bool have_target;
        float target_heading;
        uint8_t vehicle_mode;
        bool reached_alt;
    };

    void init();
    // void check_parameters();
    void update_throttle_pos();
    float get_land_airspeed();
    float stopping_distance();
    float get_holdoff_distance();
    Location get_holdoff_position();
    void check_approach_tangent();
    void check_approach_abort();
    void update_mode();
    void update_target();
    float get_target_alt();
    void update_alt();
    void update_auto_offset();
    // void handle_msg(const mavlink_message_t &msg);
    void update();

    mlstate_t.mlstate;
private:
    ;
};
