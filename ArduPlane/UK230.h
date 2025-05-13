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

    void update_target_yaw_rate();
    void update_target_bf_vel_x_ms();
    void update_target_bf_vel_y_ms();
    void update_target_ef_vel_ms();
    void update_control();

    float get_target_yaw_rate() {return _target_yaw_rate;}
    float get_target_dist_cm() {return _target_dist_cm;}
    float get_target_bf_vel_x() {return _target_bf_vel_x;}
    float get_target_bf_vel_y() {return _target_bf_vel_y;}
    float get_target_ef_vel_x() {return _target_ef_vel_x;}
    float get_target_ef_vel_y() {return _target_ef_vel_y;}

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

    FD1_UART FD1_uart_K230{AP_SerialManager::SerialProtocol_K230};
private:

    // LowPassFilterVector3f _filter_target_cm;

    // Vector3f _raw_target_cm;

    Vector3f bf_info;
    Vector3f efb_info;
    float _target_yaw_rate;
    float _target_dist_cm;
    float _target_bf_vel_x;
    float _target_bf_vel_y;
    float _target_ef_vel_x;
    float _target_ef_vel_y;
    uint32_t _last_ms;
    bool _valid;

};
