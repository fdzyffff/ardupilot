#pragma once

class UCapture {

public:

    // constructor, destructor
    UCapture();

    // initialise
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }
    int16_t cam_state();
    void init_port();
    void mav_read();
    void do_cmd(float p1);

    void handle_info(float p1, float p2, float p3, float p4);

    const Vector2f& get_raw_info() ;
    const Vector2f& get_correct_info() ;

    void update();

    float get_target_yaw_rate() {return _target_yaw_rate_cds;}
    float get_target_climb_rate() {return _target_climb_rate;}

    Vector2f raw_info;
    Vector2f correct_info;
    Vector3f current_pos;
    Vector3f target_pos;
    bool _active;
    uint32_t _last_update_ms;
    int16_t _n_count;
    int16_t _cam_state;
    float _target_yaw_rate_cds;
    float _target_climb_rate;
    float _q_cds;
    float _q_angle_cd;

private:
    void update_target_yaw_rate();
    void update_target_climb_rate();
};
