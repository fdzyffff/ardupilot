#include "FD1_message.h"

#define FD1_MSG_WAIBU_LEN 122
class FD1_msg_waibu_single : public FD1_message{
public:

    struct PACKED Target_base_t {
        uint32_t R;
        int16_t A;
        int16_t E;
        int16_t Vx;
        int16_t Vy;
        int16_t Vz;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint16_t length;
        uint8_t count;
        Time_t time;
        uint16_t control_1;
        uint16_t control_2;
        Loc_t base_loc;
        Target_base_t target[6];
        uint16_t delay_ms;
        uint8_t reserved[11];
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_WAIBU_LEN];
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
            FD1UART_PREAMBLE3,
            FD1UART_INFO,
            FD1UART_DATA,
        } msg_state;

        uint16_t read;
        uint16_t length;
        uint8_t count;
        uint8_t data[FD1_MSG_WAIBU_LEN];
    } _msg;

    FD1_msg_waibu_single();
    
    /* Do not allow copies */
    FD1_msg_waibu_single(const FD1_msg_waibu_single &other) = delete;
    FD1_msg_waibu_single &operator=(const FD1_msg_waibu_single&) = delete;

    static const uint8_t PREAMBLE1 = 0xCC;
    static const uint8_t PREAMBLE2 = 0x05;
    static const uint8_t PREAMBLE3 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
