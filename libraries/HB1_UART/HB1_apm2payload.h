#include "HB1_message.h"

#define HB1_MSG_PAYLOAD_LEN 6
class HB1_apm2payload : public HB1_message{
public:
    struct PACKED HB1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_msg_header header;
        uint8_t target;
        uint8_t cmd;
        uint8_t ret;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[HB1_MSG_PAYLOAD_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1_UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = HB1_MSG_PAYLOAD_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED HB1_UART_msg_parser
    {
        enum
        {
            HB1_UART_PREAMBLE1 = 0,
            HB1_UART_PREAMBLE2,
            HB1_UART_DATA,
            HB1_UART_SUM
        } msg_state;

        uint16_t length;
        uint16_t read_idx;
        uint8_t sum_check;
        uint8_t data[HB1_MSG_PAYLOAD_LEN];
    } _msg;

    HB1_apm2payload();
    
    /* Do not allow copies */
    HB1_apm2payload(const HB1_apm2payload &other) = delete;
    HB1_apm2payload &operator=(const HB1_apm2payload&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override {};
    void parse(uint8_t temp) override {};
    void cal_sumcheck() override;
    void swap_message() override {};

    HB1_UART_MSG_1 _msg_1;
};
