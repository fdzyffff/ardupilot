#pragma once

#include <FD1_UART/FD1_UART.h>

class Uart {

public:

    // constructor, destructor
    Uart();

    void init();
    void update();
    void read_uart();
    void write_uart();
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    FD1_msg_0919_p1& get_msg_0919_p1() { return uart_msg_0919_p1; }
    FD1_msg_0919_p2& get_msg_0919_p2() { return uart_msg_0919_p2; }
    void handle_0919_p1();
    void handle_0919_p2();
    void send_0919_p3();
    void send_0919_p4();
    void unpack_0919_lng(int32_t& lng_out, uint8_t lng_in[6]);
    void unpack_0919_lat(int32_t& lat_out, uint8_t lat_in[5]);
    void unpack_0919_alt(int32_t& alt_out, uint8_t alt_in[4]);
    void pack_0919_lng(int32_t& lng_in, uint8_t lng_out[6]);
    void pack_0919_lat(int32_t& lat_in, uint8_t lat_out[5]);
    void pack_0919_alt(int32_t& alt_in, uint8_t alt_out[4]);
    void get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out, uint16_t &second_ms_out);
    void handle_msg(const mavlink_message_t &msg);
private:

    AP_HAL::UARTDriver* _port;

    // message structure
    FD1_msg_0919_p1 uart_msg_0919_p1; //载荷至无人机 循迹移动控制指令（0xC1，0xD3）。
    FD1_msg_0919_p2 uart_msg_0919_p2; //飞行控制指令（0xC1，0xE3）描述：终端按照指令进行起飞/降落。
    FD1_msg_0919_p3 uart_msg_0919_p3; //循迹移动执行成功事件（0x75）描述：循迹移动指令回执事件。
    FD1_msg_0919_p4 uart_msg_0919_p4; //飞行控制成功事件（0x7D）描述：飞行控制回执事件。
};
