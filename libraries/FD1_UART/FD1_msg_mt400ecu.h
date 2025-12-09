#include "FD1_message.h"

#define FD1_MSG_MT400ECU_LEN 16
class FD1_msg_mt400ecu : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t pump_h;
        uint8_t pump_l;
        uint8_t test_mode;
        uint8_t spark;
        uint8_t motor;
        uint8_t rpm_h;
        uint8_t rpm_l;
        uint8_t thr_h;
        uint8_t thr_l;
        uint8_t spark_switch;
        uint8_t do_pid;
        uint8_t alt_h;
        uint8_t alt_l;
        uint8_t sum;
        uint8_t end;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_MT400ECU_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_MT400ECU_LEN;
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
        uint8_t sum;
        uint8_t data[FD1_MSG_MT400ECU_LEN];
    } _msg;

    FD1_msg_mt400ecu();
    
    /* Do not allow copies */
    FD1_msg_mt400ecu(const FD1_msg_mt400ecu &other) = delete;
    FD1_msg_mt400ecu &operator=(const FD1_msg_mt400ecu&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t POSTAMBLE1 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
