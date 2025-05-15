#include "FD1_message.h"

#define FD1_MSG_RANGER_LEN 32
class FD1_msg_RANGER : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t deviceID;
        uint8_t length;
        uint16_t error;
        int16_t high1;
        uint8_t snr1;
        int16_t speed1;
        int16_t high2;
        uint8_t snr2;
        int16_t speed2;
        int16_t high3;
        uint8_t snr3;
        int16_t speed3;
        int16_t high4;
        uint8_t snr4;
        int16_t speed4;
        int16_t high5;
        uint8_t snr5;
        int16_t speed5;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_RANGER_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_RANGER_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_ID,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_RANGER_LEN];
    } _msg;

    FD1_msg_RANGER();
    
    /* Do not allow copies */
    FD1_msg_RANGER(const FD1_msg_RANGER &other) = delete;
    FD1_msg_RANGER &operator=(const FD1_msg_RANGER&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
