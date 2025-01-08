#pragma once

class UBase {

public:

    // constructor, destructor
    UBase();

    // initialise
    void init();

    bool is_valid() const { return _valid; }
    bool new_data() {return display_info.new_data;}

    void handle_msg(const mavlink_message_t &msg);
    void set_mode(uint8_t mode_in);
    void update_target_angle();

    float get_target_pitch() {return _target_pitch;}
    float get_target_roll() {return _target_roll;}
    float get_target_yaw() {return _target_yaw;}

    void update();
    void update_valid();

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p21;
        float p22;
        float p23;
        uint16_t count;
        uint16_t count_log;
        bool new_data;
    } display_info;

private:

    bool _initialized;
    float _target_roll;
    float _target_pitch;
    float _target_yaw;
    float _base_roll;
    float _base_pitch;
    float _base_yaw;
    uint32_t _last_ms;
    bool _valid;
    // uint8_t _mode;

};
