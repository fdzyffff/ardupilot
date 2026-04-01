#pragma once

#include <FD1_UART/FD1_UART.h>

class UA8 {

public:

    // constructor, destructor
    UA8();

    // initialise
    void init();

    void read_uart();
    void handle_RK3588();
    void handle_SIYIA8mini();
    float cal_frame_angle(float angle, float x_in);
    void handle_front_info(float p1, float p2, float p3, float dist);
    void handle_up_info(float p1, float p2, float p3, float dist);

    void update_front_vel();
    void update_front_yaw_rate();
    void update_up_yaw_rate();
    void update_up_bf_vel_x_ms();
    void update_up_bf_vel_y_ms();

    void set_gimbal_front();
    void set_gimbal_up();
    void set_attitude_hz();
    bool have_target_front();
    bool have_target_up();
    uint8_t is_valid();
    float get_front_yaw_rate() {return front_status.yaw_rate;}
    float get_front_vel_x() {return front_status.vel.x;}
    float get_front_vel_y() {return front_status.vel.y;}
    float get_front_vel_z() {return front_status.vel.z;}
    float get_up_yaw_rate() {return up_status.yaw_rate;}
    float get_up_bf_vel_x() {return up_status.bf_vel.x;}
    float get_up_bf_vel_y() {return up_status.bf_vel.y;}
    Vector2f& get_front_vel_xy() {return front_status.vel.xy();}

    void update();
    void update_valid();

    void test();

    void do_print();

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p14;
        float p21;
        float p22;
        float p23;
        uint16_t count;
    } display_info;

    FD1_UART FD1_uart_RK3588{AP_SerialManager::SerialProtocol_RK3588};
    FD1_UART FD1_uart_SIYIA8{AP_SerialManager::SerialProtocol_SIYIA8};

    FD1_msg_RK3588      uart_msg_RK3588;
    FD1_msg_SIYIA8mini  uart_msg_SIYIA8mini;

private:

    // LowPassFilterVector3f _filter_target_cm;

    // Vector3f _raw_target_cm;

    struct {
        float dist_cm;
        Vector3f bf_info;
        float yaw_rate;
        Vector3f vel;
        uint32_t last_ms;
        bool valid;
        uint16_t count;
    } front_status;
    struct {
        float dist_cm;
        Vector3f bf_info;
        Vector3f efb_info;
        LowPassFilterVector3f efb_info_filt;
        float yaw_rate;
        Vector2f bf_vel;
        uint32_t last_ms;
        bool valid;
        uint16_t count;
    } up_status;
    struct {
        float zoom;
        float roll;
        float pitch;
        float yaw;
        uint32_t last_ms;
        uint32_t last_send_ms;
        bool valid;
        uint16_t count;
    } gimbal_status;

};
