#pragma once

class UGimbal {

public:

    // constructor, destructor
    UGimbal();

    // initialise
    void init();

    bool is_active() const { return _active; }
    bool new_data() {return _new_data;}
    void set_new_data(bool value) {_new_data = value;}

    void handle_info(float pitch_in, float roll_in, float yaw_in, float abs_yaw_in);
    void handle_info(int16_t tracking_status);

    const Vector3f& get_target_vel() {return _target_vel;}
    const Vector3f& get_target_pos() {return _target_pos;}
    const Vector3f& get_current_pos() {return _current_pos;}

    void update();

    Vector3f get_target_velocity() {return _target_vel;}
    float get_target_yaw_cd() {return _target_yaw_cd;}
    float get_target_dir_rate_cd() {return _target_dir_rate_cd;}

    void reset() {_dir_rate_filter.reset();}

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

    DerivativeFilterFloat_Size7         _dir_rate_filter;

    Vector3f _target_vel;
    Vector3f _target_pos;
    Vector3f _current_pos;
    float _target_yaw_cd;
    float _target_dir_rate_cd;
    uint32_t _last_ms;
    bool _active;
    bool _new_data;
};
