#pragma once

#include <FD1_UART/FD1_UART.h>

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

    void update_front_pitch_rate();
    void update_front_roll_rate();
    void update_front_yaw_rate();
    void update_up_yaw_rate();
    void update_up_bf_vel_x_ms();
    void update_up_bf_vel_y_ms();

    float set_gimbal_up();
    float set_gimbal_front();
    float have_target_up();
    float have_target_front();
    float get_front_yaw_rate() {return _front_pitch_rate;}
    float get_front_vel_x() {return _front_pitch_rate;}
    float get_front_vel_y() {return _front_pitch_rate;}
    float get_up_yaw_rate() {return _up_yaw_rate;}
    float get_up_bf_vel_x() {return _front_pitch_rate;}
    float get_up_bf_vel_y() {return _front_pitch_rate;}

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
        bool new_data;
    } display_info;

    FD1_UART FD1_uart_RK3588{AP_SerialManager::SerialProtocol_RK3588};
    FD1_UART FD1_uart_SIYIA8{AP_SerialManager::SerialProtocol_SIYIA8};

    FD1_msg_RK3588      uart_msg_RK3588;
    FD1_msg_SIYIA8mini  uart_msg_SIYIA8mini;

private:

    // LowPassFilterVector3f _filter_target_cm;

    // Vector3f _raw_target_cm;

    Vector3f bf_info;
    Vector3f efb_info;
    float _target_pitch_rate;
    float _target_roll_rate;
    float _target_yaw_rate;
    float _target_dist_cm;
    float _target_bf_vel_x;
    float _target_bf_vel_y;
    uint32_t _last_ms;
    bool _valid;

    LowPassFilterVector3f efb_info_filt;

};
