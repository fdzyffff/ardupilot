#include "FD_DYT_NET_message.h"

#define FD_DYT_NET_MSG_DYTTARGET_LEN 32
class FD_DYT_NET_msg_DYTTARGET : public FD_DYT_NET_message{
public:
    struct PACKED FD_DYT_NET_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_DYT_NET_msg_header header;
        int32_t  lat;
        int32_t  lng;
        int16_t  alt_abs;
        int16_t  alt_rel;
        uint8_t  year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  minute;
        uint8_t  second;
        uint8_t  second_10ms;
        uint8_t  reserved[10];
        uint8_t  sum_check;        //byte 3 to 41
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_DYT_NET_MSG_DYTTARGET_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_DYT_NET_MSG_DYTTARGET_LEN;
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
        uint8_t data[FD_DYT_NET_MSG_DYTTARGET_LEN];
    } _msg;

    FD_DYT_NET_msg_DYTTARGET();
    
    /* Do not allow copies */
    FD_DYT_NET_msg_DYTTARGET(const FD_DYT_NET_msg_DYTTARGET &other) = delete;
    FD_DYT_NET_msg_DYTTARGET &operator=(const FD_DYT_NET_msg_DYTTARGET&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0x18;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
