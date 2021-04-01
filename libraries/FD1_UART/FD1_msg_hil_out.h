#include "FD1_message.h"

#define FD1_MSG_HIL_OUT_LEN 27
class FD1_msg_hil_out : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t ctrl_mode;
        int16_t gyrox;
        int16_t gyroy;
        int16_t gyroz;
        int16_t accx;
        int16_t accy;
        int16_t accz;
        int16_t eulerx;
        int16_t eulery;
        int16_t eulerz;
        int16_t height_rel_home;
        int16_t velz;
        uint8_t sum_check;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_HIL_OUT_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_HIL_OUT_LEN;
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
            FD1UART_END,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_HIL_OUT_LEN];
    } _msg;

    FD1_msg_hil_out();
    
    /* Do not allow copies */
    FD1_msg_hil_out(const FD1_msg_hil_out &other) = delete;
    FD1_msg_hil_out &operator=(const FD1_msg_hil_out&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x93;
    static const uint8_t POSTAMBLE = 0xFF;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
