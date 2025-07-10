#include "FD_HaoFu_message.h"

#define FD_MSG_HaoFu_LEN 16
class FD_msg_HaoFu : public FD_HaoFu_message{
public:
    struct PACKED FD_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_msg_header header;
        float     tag_cl;
        float     tag_x;
        float     tag_y;
        uint8_t   sum;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_HaoFu_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_MSG_HaoFu_LEN;
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
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD_MSG_HaoFu_LEN];
    } _msg;

    FD_msg_HaoFu();
    
    /* Do not allow copies */
    FD_msg_HaoFu(const FD_msg_HaoFu &other) = delete;
    FD_msg_HaoFu &operator=(const FD_msg_HaoFu&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
