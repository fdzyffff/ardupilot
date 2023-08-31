#include "FD1_message.h"

#define FD1_MSG_APM2DYTTELEM_LEN 32
class FD1_msg_APM2DYTTELEM : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int32_t lat;
        int32_t lng;
        int16_t alt_abs;
        int16_t alt_rel;
        uint8_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t second_10ms;
        uint16_t airspeed;
        uint16_t gpsspeed;
        uint8_t  sum_check;  
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_APM2DYTTELEM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_APM2DYTTELEM_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_APM2DYTTELEM_LEN];
    } _msg;

    FD1_msg_APM2DYTTELEM();
    
    /* Do not allow copies */
    FD1_msg_APM2DYTTELEM(const FD1_msg_APM2DYTTELEM &other) = delete;
    FD1_msg_APM2DYTTELEM &operator=(const FD1_msg_APM2DYTTELEM&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x91;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override;

    FD1UART_MSG_1 _msg_1;
};
