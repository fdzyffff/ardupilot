#include "HB1_message.h"

class HB1_apm2mission : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t index;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        int32_t longitude;
        int32_t latitude;
        int16_t alt;
        int16_t ptich;
        int16_t roll;
        int16_t yaw;
        int16_t ground_spd;
        int16_t error_code;
        int8_t  rc_code;
        int8_t  target_wp_index;
        uint8_t console_type;
        uint8_t unused[4];
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[31];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool updated;
        bool need_send;
        const uint16_t length = 31;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*    struct PACKED HB1UART_msg_parser
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
    } _msg;*/

    HB1_apm2mission();
    
    /* Do not allow copies */
    HB1_apm2mission(const HB1_apm2mission &other) = delete;
    HB1_apm2mission &operator=(const HB1_apm2mission&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    static const uint8_t INDEX1 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override {};

    HB1UART_MSG_1 _msg_1;
};
