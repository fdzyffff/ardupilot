#include "HB1_message.h"

class HB1_apm2power : public HB1_message{
public:
    struct PACKED HB1_power2apm_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        HB1_power2apm_header header;
        uint8_t COMM1;
        uint8_t COMM2;
        uint8_t rpm_h;
        uint8_t rpm_l;
        uint8_t rel_alt;//[0~205]代表[-500,20K]m, 解析：(H - 5) * 100
        uint8_t temp;//无符号char型，[0~100]代表[-50,50]℃，ECU内部初始为15℃
        uint8_t setting_flag;//11代表设定初始温度和高度有效，22代表高度发生变化，33代表GPS不定位，00代表未设定高度。
        uint8_t airspeed;//[0~255]代表[0,510]m/s，ECU内部初始为0m/s
        uint8_t byte_11;
        uint8_t byte_22;
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

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void set_engine_start();
    void set_engine_stop();
    void set_throttle(uint8_t rpm_in);
    void make_sum();

    HB1UART_MSG_1 _msg_1;
};
