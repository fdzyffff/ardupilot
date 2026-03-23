#include "FD1_message.h"

#define FD1_MSG_HANGDONG_CAST_LEN 72
class FD1_msg_hangdong_cast : public FD1_message{
public:

    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint16_t length;
        uint8_t count;
        Time_t time;
        uint16_t status_1;
        uint16_t error_1;
        uint32_t bearing;
        int32_t yaw;
        int16_t yaw_rate;
        int32_t pitch;
        int16_t pitch_rate;
        int32_t roll;
        int32_t roll_rate;
        uint16_t airspeed;
        uint16_t ground_speed;
        uint32_t ground_course;
        Loc_t current_loc;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        int16_t yaw_corr;
        int16_t pitch_corr;
        uint8_t reserved[6];
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HANGDONG_CAST_LEN];
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
        uint8_t data[FD1_MSG_HANGDONG_CAST_LEN];
    } _msg;

    FD1_msg_hangdong_cast();
    
    /* Do not allow copies */
    FD1_msg_hangdong_cast(const FD1_msg_hangdong_cast &other) = delete;
    FD1_msg_hangdong_cast &operator=(const FD1_msg_hangdong_cast&) = delete;

    static const uint8_t PREAMBLE1 = 0xCC;
    static const uint8_t PREAMBLE2 = 0x02;
    static const uint8_t PREAMBLE3 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
