#include "FD1_message.h"

#define FD1_MSG_RK3588_LEN 40
class FD1_msg_RK3588 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t   tag_ok;
        uint32_t  tag_id;
        float     norm_x;
        float     norm_y;
        float     dist_x;
        float     dist_y;
        float     dist_z;
        float     att_roll;
        float     att_pitch;
        float     att_yaw;
        uint8_t   end;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_RK3588_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
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

        uint16_t length = FD1_MSG_RK3588_LEN;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_RK3588_LEN];
    } _msg;

    FD1_msg_RK3588();
    
    /* Do not allow copies */
    FD1_msg_RK3588(const FD1_msg_RK3588 &other) = delete;
    FD1_msg_RK3588 &operator=(const FD1_msg_RK3588&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t POSTAMBLE1 = 0xEE;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
