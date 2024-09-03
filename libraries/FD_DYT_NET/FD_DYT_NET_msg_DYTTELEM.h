#include "FD_DYT_NET_message.h"

#define FD_DYT_NET_MSG_DYTTELEM_LEN 32
class FD_DYT_NET_msg_DYTTELEM : public FD_DYT_NET_message{
public:
    struct PACKED FD_DYT_NET_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_DYT_NET_msg_header header;
        uint8_t  state1;
        uint8_t  state2;
        uint8_t  zoom;
        uint8_t  state3;
        int16_t  target_x;
        int16_t  target_y;
        int16_t  frame_roll;
        int16_t  frame_pitch;
        int16_t  frame_yaw;
        uint8_t  empty1[4];
        int16_t  roll_rate;
        int16_t  pitch_rate;
        int16_t  yaw_rate;
        uint16_t distance;
        uint8_t  check;
        uint8_t  sum_check;   
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_DYT_NET_MSG_DYTTELEM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_DYT_NET_MSG_DYTTELEM_LEN;
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
        uint8_t data[FD_DYT_NET_MSG_DYTTELEM_LEN];
    } _msg;

    FD_DYT_NET_msg_DYTTELEM();
    
    /* Do not allow copies */
    FD_DYT_NET_msg_DYTTELEM(const FD_DYT_NET_msg_DYTTELEM &other) = delete;
    FD_DYT_NET_msg_DYTTELEM &operator=(const FD_DYT_NET_msg_DYTTELEM&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0x16;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
