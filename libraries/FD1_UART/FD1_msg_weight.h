#include "FD1_message.h"

#define FD1_MSG_WEIGHT_LEN 11
class FD1_msg_weight : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t id1;
        uint16_t value1;
        uint8_t id2;
        uint16_t value2;
        uint8_t id3;
        uint16_t value3;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_WEIGHT_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_WEIGHT_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_INFO,
            FD1UART_DATA
        } msg_state;

        uint16_t read;
        uint8_t length;
        uint8_t count;
        uint8_t data[FD1_MSG_WEIGHT_LEN];
    } _msg;

    FD1_msg_weight();
    
    /* Do not allow copies */
    FD1_msg_weight(const FD1_msg_weight &other) = delete;
    FD1_msg_weight &operator=(const FD1_msg_weight&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
