#include "HB1_message.h"

class HB1_apm2power : public HB1_message{
public:
    struct PACKED HB1_mission2apm_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_mission2apm_header header;
        uint8_t ctrl_cmd;
        int16_t thr_value;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[6];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 6;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*    struct PACKED HB1UART_msg_parser
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
        uint8_t data[10];;
    } _msg;*/

    HB1_apm2power();
    
    /* Do not allow copies */
    HB1_apm2power(const HB1_apm2power &other) = delete;
    HB1_apm2power &operator=(const HB1_apm2power&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x92;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    HB1UART_MSG_1 _msg_1;
};
