#pragma once

#include <FD1_UART/FD1_msg_gimbal2gcs.h>
#include <FD1_UART/FD1_msg_gcs2gimbal.h>

class UGimbal {

public:

    // constructor, destructor
    UGimbal();

    // initialise
    void init();

    bool is_valid() const { return _valid; }
    bool new_data() {return _new_data;}
    void set_new_data(bool value) {_new_data = value;}

    void handle_info(float pitch_in, float roll_in, float yaw_in);

    float get_target_pitch_cd() {return _target_pitch_cd;}
    float get_target_roll_cd() {return _target_roll_cd;}
    float get_target_yaw_cd() {return _target_yaw_cd;}
    float get_target_climb_rate() {return _target_climb_rate;}

    void read_status_byte(uint8_t temp);
    void read_command_byte(uint8_t temp);
    void pack_msg();
    void update();
    void update_valid();

    FD1_msg_gimbal2gcs& get_msg_gimbal2gcs() { return uart_msg_gimbal2gcs; }
    FD1_msg_gcs2gimbal& get_msg_gcs2gimbal() { return uart_msg_gcs2gimbal; }
    FD1_msg_gcs2gimbal& get_msg_apm2gimbal() { return uart_msg_apm2gimbal; }

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

    LowPassFilterFloat _filter_yaw_in;

    float _target_pitch_cd;
    float _target_roll_cd;
    float _target_yaw_cd;
    float _target_climb_rate;//cm/s
    uint32_t _last_ms;
    bool _valid;
    bool _new_data;

    FD1_msg_gimbal2gcs uart_msg_gimbal2gcs;
    FD1_msg_gcs2gimbal uart_msg_gcs2gimbal; //read gcs msg
    FD1_msg_gcs2gimbal uart_msg_apm2gimbal; //copy gcs2gimbal, then add apm info, finally send to gimbal
};
