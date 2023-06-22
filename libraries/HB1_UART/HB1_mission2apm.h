#include "HB1_message.h"

class HB1_mission2apm : public HB1_message{
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
        int16_t alt;
    };

    //0x66
    struct PACKED Remote_CMD_INTERIM {
        uint8_t p1;
        uint8_t interim_point_index;
        int32_t longitude;
        int32_t latitude;
        int16_t alt;
    };

    //0x33
    struct PACKED Remote_CMD_ATTACK {
        uint8_t p1;
        uint8_t attack_point_index;
        int32_t longitude;
        int32_t latitude;
        int16_t alt;
    };

    //0x69
    struct PACKED Remote_CMD_PREATTACK {
        uint8_t time_s;
        uint8_t temp_data[11];
    };

    //0xA3
    struct PACKED Remote_CMD_AWAY {
        uint8_t temp_data[12];
    };

    //0x7E
    struct PACKED Remote_CMD_SEARCHWP {
        uint8_t line_index;
        uint8_t temp_data[11];
    };

    union PACKED Remote_CMD {
        Remote_CMD_TAKEOFF   cmd_takeoff;
        Remote_CMD_WP        cmd_wp;
        Remote_CMD_INTERIM   cmd_interim;
        Remote_CMD_ATTACK    cmd_attack;
        Remote_CMD_AWAY      cmd_away;
        Remote_CMD_PREATTACK cmd_preattack;
        Remote_CMD_SEARCHWP  cmd_searchwp;
        uint8_t cmd_data[12];
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t length;
        bool in_group;
        uint8_t remote_index;
        Remote_CMD remote_cmd;
        uint8_t control_id;
        int16_t youshang_target_airspeed;
        int16_t youshang_target_orthdist;
        int16_t youshang_target_alt;
        int16_t apm_deltaX;
        int16_t apm_deltaY;
        int16_t apm_deltaZ;
        int32_t leader_lng;
        int32_t leader_lat;
        int16_t leader_alt;
        int16_t leader_dir;
        uint8_t leader_target_id;
        bool net_timeout;
        uint8_t reserved[5]; 
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[51];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 51;
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
        uint8_t data[51];;
    } _msg;

    HB1_mission2apm();
    
    /* Do not allow copies */
    HB1_mission2apm(const HB1_mission2apm &other) = delete;
    HB1_mission2apm &operator=(const HB1_mission2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xEE;
    static const uint8_t PREAMBLE2 = 0x16;

    static const uint8_t INDEX1 = 0xAA;

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
