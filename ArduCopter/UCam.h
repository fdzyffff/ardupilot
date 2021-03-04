#pragma once

class UCam {

public:

    // constructor, destructor
    UCam();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    void handle_info(float input_x, float input_y, bool valid);

    const Vector2f& get_raw_info() const {return raw_info;}
    const Vector2f& get_correct_info() const {return correct_info;}

    void update();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate_cds;}
    float get_current_angle_deg() {return _current_angle_deg;}

private:
    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_target_track_angle();

    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    uint32_t _last_update_ms;
    float _target_pitch_rate;
    float _target_roll_angle;
    float _target_yaw_rate_cds;
    float _current_angle_deg;
};
