#include "FD1_message.h"

#define FD1_MSG_0XA2_LEN 24
class FD1_msg_0XA2 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t length;
        uint8_t ID;
        uint32_t counter;
        uint8_t status;
        int32_t baro_alt;
        int32_t airspeed; //TAS
        uint8_t cmd;
        uint8_t empty[4];
        uint16_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_0XA2_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_0XA2_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_DATA,
            FD1UART_SUM1,
            FD1UART_SUM2,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_0XA2_LEN];
    } _msg;

    FD1_msg_0XA2();
    
    /* Do not allow copies */
    FD1_msg_0XA2(const FD1_msg_0XA2 &other) = delete;
    FD1_msg_0XA2 &operator=(const FD1_msg_0XA2&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
