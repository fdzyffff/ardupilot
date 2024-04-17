#include "FD1_message.h"

#define FD1_MSG_APM2GIMBAL_LEN 44
class FD1_msg_apm2gimbal : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_APM {
        int16_t angle_pitch;// centi-degree
        int16_t angle_roll;
        uint16_t angle_yaw;//0~36000
        uint8_t gps_year;
        uint8_t gps_month;
        uint8_t gps_day;
        uint8_t gps_hour;
        uint8_t gps_minute;
        uint8_t gps_second;
        uint8_t gps_second_10ms;
        float gps_lng;
        float gps_lag;
        uint8_t gps_count;
        float gps_alt;
        uint16_t airspeed; //0.5m/s
        uint16_t relative_alt; //0.1m/s
    }

    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t empty_1[5];
        MSG_APM msg_apm;
        uint8_t empty_2[5];
        uint8_t end;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_APM2GIMBAL_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length;
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
            FD1UART_END,
        } msg_state;

        uint16_t read;
        uint8_t length;
        uint8_t count;
        uint8_t xorsum;
        uint8_t data[FD1_MSG_APM2GIMBAL_LEN];
    } _msg;

    FD1_msg_apm2gimbal();
    
    /* Do not allow copies */
    FD1_msg_apm2gimbal(const FD1_msg_apm2gimbal &other) = delete;
    FD1_msg_apm2gimbal &operator=(const FD1_msg_apm2gimbal&) = delete;

    static const uint8_t PREAMBLE1 = 0xFB;
    static const uint8_t PREAMBLE2 = 0x2C;
    static const uint8_t POSTAMBLE1 = 0xF0;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
