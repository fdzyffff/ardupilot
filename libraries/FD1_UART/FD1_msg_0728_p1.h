#include "FD1_message.h"

#define FD1_MSG_0728_P1_LEN 31
class FD1_msg_0728_p1 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t length;
        uint8_t count;
        uint8_t recieve_id;
        uint8_t command_id;
        uint32_t wp_lng; // 1e7
        uint32_t wp_lat;
        uint16_t wp_alt;
        uint16_t speed;
        int32_t pitch;
        int32_t roll;
        int32_t yaw;
        uint8_t sum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_0728_P1_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_0728_P1_LEN;
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
        uint8_t length;
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_0728_P1_LEN];
    } _msg;

    FD1_msg_0728_p1();
    
    /* Do not allow copies */
    FD1_msg_0728_p1(const FD1_msg_0728_p1 &other) = delete;
    FD1_msg_0728_p1 &operator=(const FD1_msg_0728_p1&) = delete;

    static const uint8_t PREAMBLE1 = 0xBB;
    static const uint8_t PREAMBLE2 = 0x7E;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
