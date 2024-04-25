#include "HB1_message.h"

class HB1_power2apm : public HB1_message{
public:
    struct PACKED HB1_2apm_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_2apm_header header;
        uint8_t byte_3;
        uint8_t cmd_back;
        uint8_t ctrl_rpm_h;
        uint8_t ctrl_rpm_l;
        uint8_t temp_h;
        uint8_t temp_l;
        uint8_t byte_9;
        uint8_t byte_10;
        uint8_t pump_volt;
        uint8_t main_pwm;
        uint8_t sub_pwm;
        uint8_t motor_pwm;
        uint8_t switch_back;
        uint8_t rpm_h;
        uint8_t rpm_l;
        uint8_t throttle;
        uint8_t save_flag;
        uint8_t check;
        uint8_t status;
        uint8_t error_code;
        uint8_t fps_count;
        uint8_t temp_ecu;
        uint8_t cmd_rpm_h;
        uint8_t cmd_rpm_l;
        uint8_t byte_27;
        uint8_t byte_28;
        uint8_t sum;
        uint8_t xorsum;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[14];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED HB1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = 14;
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
            HB1UART_XOR,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t sum_xor;
        HB1_2apm_header header;
        uint8_t data[14];;
    } _msg;

    HB1_power2apm();
    
    /* Do not allow copies */
    HB1_power2apm(const HB1_power2apm &other) = delete;
    HB1_power2apm &operator=(const HB1_power2apm&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    HB1UART_MSG_1 _msg_1;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
};
