#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define NUM_MY_DATA 5

class UserUartFWD;

class User_data_buffer {
public:
    User_data_buffer() {}
    void set_active();
    void update() ;
    uint16_t get_data(uint8_t (&data)[200]);
    void push(uint8_t c);

    bool _active;
    uint32_t _last_active_ms;
    uint16_t data_idx;
    uint8_t _data[500];
    uint8_t _id;
};

class UserUartFWD {
    friend class User_data_buffer;
public:
    UserUartFWD(enum AP_SerialManager::SerialProtocol protocol);
    void init();
    void handle_msg(const mavlink_message_t &msg);
    void update();
    void read_uart() ;
    void send_mav();
    void set_target_sysid(uint16_t id_in);

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;
    uint8_t _target_sys_id;

    User_data_buffer data_buffer_instance[NUM_MY_DATA];
};
