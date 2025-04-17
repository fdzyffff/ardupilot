#include "HB1_message.h"

class HB1_apm2power : public HB1_message{
public:
    struct PACKED HB1_power2apm_header {
        uint8_t head_1;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_power2apm_header header;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t crc;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[4];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 4;
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
        HB1_power2apm_header header;
        uint8_t data[6];;
    } _msg;*/

    HB1_apm2power();
    
    /* Do not allow copies */
    HB1_apm2power(const HB1_apm2power &other) = delete;
    HB1_apm2power &operator=(const HB1_apm2power&) = delete;

    static const uint8_t PREAMBLE1 = 0xFF;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void set_engine_start();
    void set_engine_stop();
    void set_engine_throttle_control(uint16_t thr_in);
    void set_engine_emergency_stop();
    void set_throttle(uint8_t thr_in);
    void set_rpm_half(uint16_t rpm_in);
    void make_sum();

    HB1UART_MSG_1 _msg_1;
};
