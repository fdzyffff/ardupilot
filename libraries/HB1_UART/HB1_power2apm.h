#include "HB1_message.h"

class HB1_power2apm : public HB1_message{
public:
    struct PACKED HB1_2apm_header {
        uint8_t head_1;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_2apm_header header;
        uint8_t lsb;
        uint8_t msb;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t xorsum;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[7];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 7;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED HB1UART_msg_parser
    {
        enum
        {
            HB1UART_PREAMBLE1 = 0,
            //HB1UART_INDEX,
            HB1UART_DATA,
            HB1UART_SUM,
        } msg_state;

        uint16_t length = 7;
        uint16_t read;
        HB1_2apm_header header;
        uint8_t data[7];;
    } _msg;

    HB1_power2apm();
    
    /* Do not allow copies */
    HB1_power2apm(const HB1_power2apm &other) = delete;
    HB1_power2apm &operator=(const HB1_power2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xF1;

    HB1UART_MSG_1 _msg_1;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
};
