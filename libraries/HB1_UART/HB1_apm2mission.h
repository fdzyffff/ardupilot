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
        uint8_t length;
        // uint8_t remote_index;
        // uint8_t line_index;
        // uint8_t point_index;
        int32_t longitude;
        int32_t latitude;
        int16_t alt;
        // uint8_t control_id;
        int16_t ptich;
        int16_t roll;
        int16_t yaw;
        int16_t air_speed;
        uint8_t error_code1;
        uint8_t error_code2;
        int8_t  rc_code;
        int8_t  target_wp_index;
        bool in_group;
        int16_t gspd;
        int16_t gspd_dir;
        uint8_t mission_state;
        uint8_t control_type;
        // int16_t target_dist;
        // uint8_t target_control_id;
        uint8_t reserved[2];
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[36];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 36;
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
        uint8_t data[41];;
    } _msg;*/

    HB1_apm2mission();
    
    /* Do not allow copies */
    HB1_apm2mission(const HB1_apm2mission &other) = delete;
    HB1_apm2mission &operator=(const HB1_apm2mission&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0x16;

    static const uint8_t INDEX1 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override {};

    const double SF_LL = 1.19304647056;
    const float SF_ALT = 3.2767;
    const float SF_DIST = 32.767;
    const float SF_ANG = 182.0389;
    const float SF_VEL = 327.67;

    HB1UART_MSG_1 _msg_1;
};
