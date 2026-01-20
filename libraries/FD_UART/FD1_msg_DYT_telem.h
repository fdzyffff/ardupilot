#include "FD1_message.h"

#define FD1_MSG_DYT_TELEM_LEN 31
class FD1_msg_DYT_telem : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t status_1;
        uint8_t status_2;
        uint8_t zoom;
        uint8_t status_3;
        int16_t target_yaw;
        int16_t target_pitch;
        int16_t gimbal_roll;
        int16_t gimbal_pitch;
        int16_t gimbal_yaw;
        uint8_t frame_pixel_x;
        uint8_t frame_pixel_y;
        uint8_t reserved_1[2];
        int16_t roll_rate;
        int16_t pitch_rate;
        int16_t yaw_rate;
        uint16_t dist;
        uint8_t self_check;
        uint8_t reserved_2[2];
        uint8_t sum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_DYT_TELEM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_DYT_TELEM_LEN;
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
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        uint16_t length;
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_DYT_TELEM_LEN];
    } _msg;

    FD1_msg_DYT_telem();
    
    /* Do not allow copies */
    FD1_msg_DYT_telem(const FD1_msg_DYT_telem &other) = delete;
    FD1_msg_DYT_telem &operator=(const FD1_msg_DYT_telem&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0x16;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
