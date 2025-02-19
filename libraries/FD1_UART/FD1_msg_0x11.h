#include "FD1_message.h"

#define FD1_MSG_0X11_LEN 55
class FD1_msg_0x11 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint16_t length;
        uint8_t cmd_type;
        float gps_lng;
        float gps_lat;
        uint16_t relative_alt;
        uint16_t absolute_alt;
        uint16_t baro_alt;
        int16_t pitch_angle;
        int16_t roll_angle;
        uint16_t yaw_angle;
        int16_t airspeed;
        int16_t vel_n;
        int16_t vel_e;
        int16_t vel_d;
        uint8_t rest_time;
        uint8_t status;
        uint8_t gps_count;
        uint8_t pos_source;
        uint8_t flight_mode;
        uint32_t time2000;
        int16_t vel_lat;
        int16_t vel_lng;
        int16_t vel_alt;
        int16_t pitch_rate;
        int16_t roll_rate;
        int16_t yaw_rate;
        uint8_t xorsum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_0X11_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_0X11_LEN;
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
        uint8_t xorsum;
        uint8_t data[FD1_MSG_0X11_LEN];
    } _msg;

    FD1_msg_0x11();
    
    /* Do not allow copies */
    FD1_msg_0x11(const FD1_msg_0x11 &other) = delete;
    FD1_msg_0x11 &operator=(const FD1_msg_0x11&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_LNG = 180.f/(2e31-1.f);
    const float SF_LAT = 90.f/(2e31-1.f);
};
