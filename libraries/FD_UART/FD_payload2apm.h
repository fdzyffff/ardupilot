#include "FD_message.h"

#define FD_MSG_PAYLOADAPM_LEN 6
class FD_payload2apm : public FD_message{
public:
    struct PACKED FD_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_msg_header header;
        uint8_t target;
        uint8_t cmd;
        uint8_t ret;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_PAYLOADAPM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD_UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_MSG_PAYLOADAPM_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD_UART_msg_parser
    {
        enum
        {
            FD_UART_PREAMBLE1 = 0,
            FD_UART_PREAMBLE2,
            FD_UART_DATA,
            FD_UART_SUM
        } msg_state;

        uint16_t length;
        uint16_t read_idx;
        uint8_t sum_check;
        uint8_t data[FD_MSG_PAYLOADAPM_LEN];
    } _msg;

    FD_payload2apm();
    
    /* Do not allow copies */
    FD_payload2apm(const FD_payload2apm &other) = delete;
    FD_payload2apm &operator=(const FD_payload2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void cal_sumcheck() override {};
    void swap_message() override {};

    FD_UART_MSG_1 _msg_1;
};
