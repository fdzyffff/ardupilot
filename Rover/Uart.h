#pragma once

class Uart {

public:

    // constructor, destructor
    Uart();

    // initialise
    void init();

    void read_uart();
    void write_uart();
    uint8_t cal_bearing();
    uint8_t cal_speed();
    void update();
    AP_HAL::UARTDriver* get_port(void) {return _port;}


private:

    // message structure
    struct PACKED MSG_Command_1 {
        uint8_t header;
        uint8_t bearing;//范围：0 - 180（90度正前）
        uint8_t speed;//范围：0 - 200（100速度0）
        uint8_t reserved[4];
        uint8_t accumulate;
    };

    union PACKED UART_MSG {
        MSG_Command_1 msg;
        uint8_t data[8];
    };

    static const uint8_t PREAMBLE1 = 0x58;

    UART_MSG _msg;

    AP_HAL::UARTDriver* _port;
};
