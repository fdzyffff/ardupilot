#pragma once

class UBase {

public:

    // constructor, destructor
    UBase();

    // initialise
    void init();

    bool is_active() const { return _active; }
    bool new_data() {return _new_data;}
    void handle_msg(const mavlink_message_t &msg);

    void handle_info(Location target_loc_in, Vector3f target_pos_in, Vector3f target_vel_in, float target_heading_in);

    Location& get_target_loc() {return _target_loc;}
    Vector3f get_target_pos() {return _target_pos;}
    Vector3f get_target_vel() {return _target_vel;}
    float get_target_bearing() {return _target_bearing;}

    void update();

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

private:

    Location _target_loc;
    Vector3f _target_pos;
    Vector3f _target_vel;
    float _target_bearing;

    uint32_t _last_ms;
    bool _active;
    bool _new_data;
};
