#include "HB1_message.h"

class HB1_cam2mission : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t index;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t length;
        uint8_t uav_id;
        uint8_t unread[45];
        uint8_t unused;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[52];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 52;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED HB1UART_msg_parser
    {
        enum
        {
            HB1UART_PREAMBLE1 = 0,
            HB1UART_PREAMBLE2,
            HB1UART_INDEX,
            HB1UART_DATA,
            HB1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        HB1_mission2apm_header header;
        uint8_t data[60];;
    } _msg;

    HB1_cam2mission();
    
    /* Do not allow copies */
    HB1_cam2mission(const HB1_cam2mission &other) = delete;
    HB1_cam2mission &operator=(const HB1_cam2mission&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    
    static const uint8_t INDEX1 = 0xB4;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override {};

    HB1UART_MSG_1 _msg_1;
};
