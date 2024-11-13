#include "FD_message.h"

#define FD_CAM_TARGET_LEN 13
class FD_CAM_TARGET : public FD_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t on;
        uint8_t status;
        int16_t target_x;
        int16_t target_y;
        int16_t target_w;
        int16_t target_h;
        uint8_t sum_check;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_CAM_TARGET_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length;
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
            FD1UART_SUM
        } msg_state;

        uint16_t read;
        uint8_t length;
        uint8_t count;
        uint8_t sum_check;
        uint8_t data[FD_CAM_TARGET_LEN];
    } _msg;

    FD_CAM_TARGET();
    
    /* Do not allow copies */
    FD_CAM_TARGET(const FD_CAM_TARGET &other) = delete;
    FD_CAM_TARGET &operator=(const FD_CAM_TARGET&) = delete;

    static const uint8_t PREAMBLE1 = 0x91;
    static const uint8_t PREAMBLE2 = 0x0D;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
