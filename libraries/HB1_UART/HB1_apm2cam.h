#include "HB1_message.h"

class HB1_apm2cam : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t position_status;
        uint8_t gps_year;
        uint8_t gps_month;
        uint8_t gps_day;
        uint8_t gps_hour;
        uint8_t gps_minute;
        uint8_t gps_second;
        uint16_t gps_millis;
        int32_t longitude;
        int32_t latitude;
        int16_t gps_alt;
        int16_t ground_spd;
        int16_t ptich_cd;
        int16_t roll_cd;
        int16_t yaw_cd;
        uint16_t air_speed;
        int16_t baro_alt;
        uint16_t gps_yaw;
        uint8_t gps_nstats;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[37];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool updated;
        bool need_send;
        const uint16_t length = 37;
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

    HB1_apm2cam();
    
    /* Do not allow copies */
    HB1_apm2cam(const HB1_apm2cam &other) = delete;
    HB1_apm2cam &operator=(const HB1_apm2cam&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x91;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    HB1UART_MSG_1 _msg_1;
};
