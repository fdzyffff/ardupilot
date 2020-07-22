#include "HB1_message.h"

class HB1_mission2apm_v1 : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t index;
    };

    //0x63
    struct PACKED Remote_CMD_TAKEOFF {
        uint8_t temp_data[12];
    };

    //0x9C
    struct PACKED Remote_CMD_WP {
        uint8_t line_index;
        uint8_t point_index;
        int32_t longitude;
        int32_t latitude;
        uint16_t alt;
    };

    //0x66
    struct PACKED Remote_CMD_INTERIM {
        uint8_t p1;
        uint8_t interim_point_index;
        int32_t longitude;
        int32_t latitude;
        uint16_t alt;
    };

    //0x33
    struct PACKED Remote_CMD_ATTACK {
        uint8_t p1;
        uint8_t attack_point_index;
        int32_t longitude;
        int32_t latitude;
        uint16_t alt;
    };

    //0xA3
    struct PACKED Remote_CMD_AWAY {
        uint8_t temp_data[12];
    };

    union PACKED Remote_CMD {
        Remote_CMD_TAKEOFF  cmd_takeoff;
        Remote_CMD_WP       cmd_wp;
        Remote_CMD_INTERIM  cmd_interim;
        Remote_CMD_ATTACK   cmd_attack;
        Remote_CMD_AWAY     cmd_away;
        uint8_t cmd_data[12];
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t console_type;
        uint8_t remote_index;
        Remote_CMD remote_cmd;
        uint8_t unused[4];
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[22];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 22;
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

    HB1_mission2apm_v1();
    
    /* Do not allow copies */
    HB1_mission2apm_v1(const HB1_mission2apm_v1 &other) = delete;
    HB1_mission2apm_v1 &operator=(const HB1_mission2apm_v1&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    static const uint8_t INDEX1 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override {};

    HB1UART_MSG_1 _msg_1;
};
