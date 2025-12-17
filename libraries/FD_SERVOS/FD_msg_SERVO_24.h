#include "FD_SERVO_message.h"

#define FD_MSG_SERVO_24_LEN 9
class FD_msg_SERVO_24 : public FD_SERVO_message{
public:
    struct PACKED FD_msg_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t data_id;
        uint8_t data_length;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_msg_header header;
        uint8_t id;
        uint8_t method;
        uint16_t Power;
        uint8_t sum_check; // (包头所有字节 + 数据内容所有字节) % 256。
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_SERVO_24_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_SERVO_24_LEN;
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
        uint8_t data[FD_MSG_SERVO_24_LEN];
    } _msg;

    FD_msg_SERVO_24();
    
    /* Do not allow copies */
    FD_msg_SERVO_24(const FD_msg_SERVO_24 &other) = delete;
    FD_msg_SERVO_24 &operator=(const FD_msg_SERVO_24&) = delete;

    static const uint8_t PREAMBLE1 = 0x12;
    static const uint8_t PREAMBLE2 = 0x4C;
    static const uint8_t DATA_ID = 24;
    static const uint8_t DATA_LENGTH = 4;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check();

    FD1UART_MSG_1 _msg_1;
};
