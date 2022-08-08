#pragma once

class UCam {

public:

    // constructor, destructor
    UCam();

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

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate_cds;}
    float get_current_angle_deg() {return _current_angle_deg;}

    bool display_info_new;
    float display_info_p1;
    float display_info_p2;
    float display_info_p3;
    float display_info_p4;
    int16_t display_info_count;
private:
    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_target_track_angle();


    AP_HAL::UARTDriver* get_port(void) {return _port;}

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    Vector2f raw_info;
    Vector2f correct_info;
    bool _active;
    uint32_t _last_update_ms;
    int16_t _n_count;
    int16_t _cam_state;
    bool _new_data;
    AP_HAL::UARTDriver* _port;
    float _target_pitch_rate;
    float _target_roll_angle;
    float _target_yaw_rate_cds;
    float _current_angle_deg;
};
