#include "FD1_message.h"

#define FD1_MSG_0919_P1_LEN 271
class FD1_msg_0919_p1 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED FD1_msg_content {
        uint8_t wp_lng[6];
        uint8_t wp_lat[5];
        uint8_t wp_alt[4];
        uint8_t wp_spd;
    };

    union PACKED FD1_msg_wp {
        FD1_msg_content content_wp;
        uint8_t data[16];
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t length;
        uint8_t T_Type;
        uint8_t T_Subtype;
        uint8_t RID;
        uint16_t PID;
        uint8_t T_Type_c;
        uint8_t T_Subtype_c;
        uint8_t RID_c;
        uint16_t PID_c;
        uint8_t wp_number;
        FD1_msg_wp wp_data[16];
        uint8_t sum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_0919_P1_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_0919_P1_LEN;
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
        uint8_t data[FD1_MSG_0919_P1_LEN];
    } _msg;

    FD1_msg_0919_p1();
    
    /* Do not allow copies */
    FD1_msg_0919_p1(const FD1_msg_0919_p1 &other) = delete;
    FD1_msg_0919_p1 &operator=(const FD1_msg_0919_p1&) = delete;

    static const uint8_t PREAMBLE1 = 0xC1;
    static const uint8_t PREAMBLE2 = 0xD3;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
