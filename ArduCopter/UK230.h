#pragma once

#include <FD_UART/FD_UART.h>

class UK230 {

public:

    // constructor, destructor
    UK230();

    // initialise
    void init();

    bool is_valid() const { return _valid; }
    bool new_data() {return display_info.new_data;}

    void read_uart();
    float cal_frame_angle(float pixel, float angle, float x_in);
    void handle_info(float p1, float p2, float p3);

    void update();
    void update_valid();
    void update_print();
    void update_target_pitch_rate();
    void update_target_roll_rate();
    void update_target_yaw_rate();
    void update_target_bf_vel_x_ms();
    void update_target_bf_vel_y_ms();
    void update_target_ef_vel_ms();
    void update_log();

    float get_target_dist_cm() {return _target_dist_cm;}
    float get_target_bf_vel_x() {return _target_bf_vel_x;}
    float get_target_bf_vel_y() {return _target_bf_vel_y;}
    float get_target_ef_vel_x() {return _target_ef_vel_x;}
    float get_target_ef_vel_y() {return _target_ef_vel_y;}

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
        bool new_data;
    } display_info;

    AP_HAL::UARTDriver* get_port(void) {return _port;}
private:

    AP_HAL::UARTDriver* _port;

    Vector3f bf_info;
    Vector3f ebf_info;
    float _target_dist_cm;
    float _target_bf_vel_x;
    float _target_bf_vel_y;
    float _target_ef_vel_x;
    float _target_ef_vel_y;
    uint32_t _last_ms;
    bool _valid;

    // message structure
    FD_msg_K230 msg_k230;
};
