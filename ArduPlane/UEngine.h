#pragma once

#include <FD1_UART/FD1_UART.h>

#define UENGINE_MAX_NUM 5

class UEngines;

class UEngine {

public:

    // constructor, destructor
    UEngine(UEngines *fronted_in, uint8_t id_in);

    void init();
    void update();
    void read_uart();
    void write_uart();
    void set_rpm(uint16_t rpm_in);
    void send_request();
    void send_mavlink_msg(mavlink_channel_t chan);

    AP_HAL::UARTDriver* get_port(void) {return _port;}


private:
    UEngines *_fronted;
    uint8_t _id;

    AP_HAL::UARTDriver* _port;

    mavlink_hxts_hy_engine_t hxts_hy_engine_packet;
    // message structure
    FD1_msg_engine_send uart_engine_send;
    FD1_msg_engine_response uart_engine_response;
};

class UEngines {

public:

    // constructor, destructor
    UEngine();

    void init();
    void update();
    void set_rpm(uint8_t id_in, uint16_t rpm_in);

    UEngine engines[UENGINE_MAX_NUM];

};
