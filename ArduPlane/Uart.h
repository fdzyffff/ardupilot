#pragma once

#include <FD_UART/FD_UART.h>

class Uart {

public:

    // constructor, destructor
    Uart();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    void read_uart();
    void write_uart();
    void handle_LS_control();
    void update_status();
    void do_print();
    AP_HAL::UARTDriver* get_port(void) {return _port;}
    
    void set_target_angle(float gimbal_yaw, float gimbal_pitch);
    void set_target_loc(Location& loc_in);
    void pack_status();

    struct {
        bool valid;
        uint8_t type;
        uint8_t cmd;
        uint32_t last_cmd_ms;
        Location cmd_loc;
        float cmd_speed;
        float cmd_alt;
        float cmd_pitch;
        float cmd_roll;
    } control_status;

private:

    AP_Int16        print;

    AP_HAL::UARTDriver* _port;

    struct {
        uint32_t last_status_send_ms;
        float gimbal_yaw;
        float gimbal_pitch;
        Location target_loc;
    } send_status;

    // message structure
    FD1_msg_LS_control uart_msg_LS_control;
    FD1_msg_LS_status uart_msg_LS_status;
};
