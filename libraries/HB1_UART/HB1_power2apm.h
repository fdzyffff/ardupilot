#include "HB1_message.h"

class HB1_power2apm : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t batt_currstatus;
        uint8_t batt_voltstatus;
        uint16_t CYS350_rpm;
        int16_t CYS350_temp;
        uint8_t FQ340_status;
        uint16_t FQ340_rpm;
        int8_t FQ340_airtemp;
        uint8_t FQ340_pluswidth;
        uint16_t FQ340_throttle;
        uint8_t FQ340_ECUvolt;
        uint16_t FQ340_airpressure;
        uint16_t FQ340_fuelpressure;
        int16_t FQ340_temp1;
        int16_t FQ340_temp2;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[25];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 25;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED HB1UART_msg_parser
    {
        enum
        {
            HB1UART_PREAMBLE1 = 0,
            HB1UART_PREAMBLE2,
            //HB1UART_INDEX,
            HB1UART_DATA,
            HB1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        HB1_mission2apm_header header;
        uint8_t data[60];;
    } _msg;

    HB1_power2apm();
    
    /* Do not allow copies */
    HB1_power2apm(const HB1_power2apm &other) = delete;
    HB1_power2apm &operator=(const HB1_power2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x93;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    HB1UART_MSG_1 _msg_1;
};
