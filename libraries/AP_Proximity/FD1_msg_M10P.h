#include "FD1_message.h"

#define FD1_MSG_M10P_LEN 160
class FD1_msg_M10P : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint16_t length;
        uint16_t angle;
        uint16_t speed;
        uint8_t dist[140];
        uint8_t gps_time[10];
        uint8_t end_1;
        uint8_t end_2;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_M10P_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_M10P_LEN;
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
            FD1UART_POSTAMBLE1,
            FD1UART_POSTAMBLE2,
        } msg_state;

        uint16_t length = FD1_MSG_M10P_LEN;
        uint16_t read;
        uint8_t data[FD1_MSG_M10P_LEN];
    } _msg;

    FD1_msg_M10P();
    
    /* Do not allow copies */
    FD1_msg_M10P(const FD1_msg_M10P &other) = delete;
    FD1_msg_M10P &operator=(const FD1_msg_M10P&) = delete;

    static const uint8_t PREAMBLE1 = 0xA5;
    static const uint8_t PREAMBLE2 = 0x5A;
    static const uint8_t POSTAMBLE1 = 0xFA;
    static const uint8_t POSTAMBLE2 = 0xFB;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
