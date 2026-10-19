#pragma once

#include <FD_UART/FD_UART.h>

// 时统 GPIO 由参数 PPS_OUT_PIN / SYNC_IN_PIN 配置；对应 SERIALx_PROTOCOL 必须为 -1

class Uart {

public:

    // constructor, destructor
    Uart();

    void init();
    void update();
    void update_status();
    void read_uart();
    void write_uart();
    void do_print();
    void read_byte(uint8_t temp);
    void send_reply(uint8_t cmd_type);
    void handle_info_diaocang();
    void handle_info_huokong();
    void handle_info_shuangmu();
    void handle_info_waibu();
    void send_hangdong();
    void request_send();
    void check_time_scale();
    void pps_tick();
    void sync_irq(uint8_t pin, bool state, uint32_t timestamp);
    bool sync_valid() const;
    void get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out);
    uint32_t get_time_ms(FD1_message::Time_t time);
    void set_tims_ms(FD1_message::Time_t &time);
    AP_HAL::UARTDriver* get_port(void) {return _port;}

private:

    AP_HAL::UARTDriver* _port;

    uint32_t cali_last_ms;
    uint32_t cali_last_ms_global;
    uint32_t time_check_ms;
    uint32_t time_gps_sec_last;
    uint32_t time_edge_local;
    uint32_t time_edge_count;
    int32_t time_edge_err_sum;

    uint16_t pps_count;
    int16_t pps_gpio;
    int16_t sync_gpio;
    bool timesync_printed;
    volatile bool sync_pending;
    volatile uint32_t sync_last_us;
    volatile uint32_t sync_count;

    struct {
        uint32_t last_diaocang_ms;
        uint32_t last_huokong_ms;
        uint32_t last_huokong_ok_ms;
        uint32_t last_shuangmu_ms;
        uint32_t last_waibu_ms;
        bool valid_diaocang;
        bool valid_huokong;
        bool valid_huokong_ok;
        bool valid_shuangmu;
        bool valid_waibu;
        uint16_t count_diaocang;
        uint16_t count_huokong;
        uint16_t count_shuangmu;
        uint16_t count_waibu;
        uint8_t target_diaocang;
    } status;

    // message structure
    FD1_msg_diaocang_cast uart_msg_diaocang_cast; //吊舱点播发送的报文内容
    FD1_msg_hangdong_cast uart_msg_hangdong_cast; //航动点播发送的报文内容
    FD1_msg_huokong_cast uart_msg_huokong_cast;   //火控点播发送的报文内容
    FD1_msg_shuangmu_cast uart_msg_shuangmu_cast; //双目数据组播发送的报文内容
    FD1_msg_waibu_single uart_msg_waibu_single; //地面终端对外接收信息内容
};
