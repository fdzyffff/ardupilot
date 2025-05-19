#include "FD1_message.h"

#define FD1_MSG_PAYLOAD_LEN 7
class FD1_msg_Payload : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header; //同步头
        uint8_t type;
        uint8_t cmd;
        uint8_t sum;
        uint8_t post1;
        uint8_t post2;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_PAYLOAD_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_PAYLOAD_LEN;
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
            FD1UART_SUM,
            FD1UART_POSTAMBLE1,
            FD1UART_POSTAMBLE2,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_PAYLOAD_LEN];
    } _msg;

    FD1_msg_Payload();
    
    /* Do not allow copies */
    FD1_msg_Payload(const FD1_msg_Payload &other) = delete;
    FD1_msg_Payload &operator=(const FD1_msg_Payload&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t POSTAMBLE1 = 0x50;
    static const uint8_t POSTAMBLE2 = 0xFC;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
