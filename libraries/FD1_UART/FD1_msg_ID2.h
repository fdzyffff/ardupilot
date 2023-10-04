#include "FD1_message.h"

#define FD1_MSG_ID2_LEN 16
class FD1_msg_ID2 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t  type;
        uint8_t  count;
        uint8_t  length;
        int32_t  lat; //1e7
        int32_t  lng; //1e7
        int16_t  alt; //meter
        uint8_t  sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_ID2_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_ID2_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_PRETYPE,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_ID2_LEN];
    } _msg;

    FD1_msg_ID2();

    /* Do not allow copies */
    FD1_msg_ID2(const FD1_msg_ID2 &other) = delete;
    FD1_msg_ID2 &operator=(const FD1_msg_ID2&) = delete;

    static const uint8_t PREAMBLE1 = 0xFE;
    static const uint8_t PREAMBLE2 = 0x07;
    static const uint8_t PRETYPE = 0x02;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void cal_sumcheck() override;

    FD1UART_MSG_1 _msg_1;
};
