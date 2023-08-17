#pragma once

class UAttack_Port;

class UAttack {

public:

    // constructor, destructor
    UAttack();

    void init();
    bool is_active() const { return _active; }
    void udpate_value();
    void init_cam_port();
    void update();
    void do_cmd(float p1, float p2, float p3, float p4);
    const Vector2f& get_raw_info();
    const Vector2f& get_correct_info();

    float get_target_pitch_rate_cds() {return _target_pitch_rate_cds;}
    float get_target_roll_angle_cd() {return _target_roll_angle_cd;}
    float get_target_yaw_rate() {return _target_yaw_rate_cds;}

    AP_HAL::UARTDriver* get_port(void) {return _cam_port;}

    bool display_info_new;
    float display_info_p1;
    float display_info_p2;
    float display_info_p3;
    float display_info_p4;
    int16_t display_info_count;

    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    AP_HAL::UARTDriver* _cam_port;
    float _target_pitch_rate_cds;
    float _target_roll_angle_cd;
    float _target_yaw_rate_cds;

    UCam* _UCam_ptr;
    uint8_t _cam_port_type;


private:
    void cam_update();
    void time_out_check();
    void update_target_pitch_rate_cds();
    void update_target_roll_angle_cd();
    void update_target_yaw_rate_cd();
};
