#include "FD1_message.h"

#define FD1_MSG_STATUS_LEN 35
class FD1_msg_status : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t head_3;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint32_t sys_time;
        uint8_t gps_ok;
        int32_t lng;
        int32_t lat;
        uint16_t alt;
        int32_t launch_lng;
        int32_t launch_lat;
        uint16_t launch_alt;
        uint8_t reserved[6];
        uint8_t sum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_STATUS_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        int32_t length = FD1_MSG_STATUS_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_PREAMBLE3,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        int32_t length;
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_STATUS_LEN];
    } _msg;

    FD1_msg_status();
    
    /* Do not allow copies */
    FD1_msg_status(const FD1_msg_status &other) = delete;
    FD1_msg_status &operator=(const FD1_msg_status&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0xAA;
    static const uint8_t PREAMBLE3 = 0x1F;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
