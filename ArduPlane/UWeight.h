#pragma once

#include <FD1_UART/FD1_UART.h>

class UWeight {

public:

    // constructor, destructor
    UWeight();

    void init();
    void update();
    void read_uart();
    void write_uart();
    void check_alive();
    void set_rpm(uint16_t rpm_in);
    void send_request();
    void send_mavlink_msg(mavlink_channel_t chan);
    void handle_message(const mavlink_message_t &msg);
    void do_print();
    void update_log();
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_HAL::UARTDriver* get_port(void) {return _port;}

    struct {
        bool new_data;
    } display_info;

private:

    AP_Int16        print;

    uint32_t _last_update_ms;
    uint32_t _last_log_ms;
    bool _alive;

    AP_HAL::UARTDriver* _port;

    mavlink_hxts_hy_weight_t hxts_hy_weight_packet;
    // message structure
    FD1_msg_weight uart_msg_weight;
};
