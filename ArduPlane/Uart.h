#pragma once

#include <FD_UART/FD_UART.h>

class Uart {

public:

    // constructor, destructor
    Uart();

    void init();
    void update();
    void read_uart();
    void write_uart();
    void read_byte(uint8_t temp);
    void send_reply(uint8_t cmd_type);
    void handle_info_diaocang();
    void handle_info_huokong();
    void handle_info_shuangmu();
    void handle_info_waibu();
    void send_hangdong();
    uint32_t get_time_ms(FD1_message::Time_t time);
    void set_tims_ms(FD1_message::Time_t &time);
    AP_HAL::UARTDriver* get_port(void) {return _port;}

private:

    AP_HAL::UARTDriver* _port;

    uint32_t cali_last_ms;
    uint32_t cali_last_ms_global;

    // message structure
    FD1_msg_diaocang_cast uart_msg_diaocang_cast; //吊舱数据组播发送的报文内容
    FD1_msg_hangdong_cast uart_msg_hangdong_cast; //航动组播发送的报文内容
    FD1_msg_huokong_cast uart_msg_huokong_cast; //火控组播发送的报文内容
    FD1_msg_shuangmu_cast uart_msg_shuangmu_cast; //双目数据组播发送的报文内容
    FD1_msg_waibu_single uart_msg_waibu_single; //地面终端对外接收信息内容
};
