#include "FD_message.h"

#define FD_MSG_ENGINE_LEN 7
class FD_engine : public FD_message{
public:
    // message structure
    struct PACKED Msg_err {
        uint8_t code;
        uint8_t temperature;
        uint8_t throttle;
        uint8_t voltage_pump;
    };

    struct PACKED Msg_attach {
        uint8_t voltage_idle;
        uint8_t voltage_max;
        uint8_t reserved;
        uint8_t voltage_batt;
    };

    struct PACKED Msg_common {
        uint8_t rpm;
        uint8_t temperature;
        uint8_t throttle;
        uint8_t voltage_pump;
    };

    union PACKED Msg_sub {
        Msg_err msg_err;
        Msg_attach msg_attach;
        Msg_common msg_common;
        uint8_t data[4];
    };

    struct PACKED MSG_Command_1 {
        uint8_t header;
        uint8_t id;
        uint8_t status;
        Msg_sub msg_sub;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_ENGINE_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD_UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_MSG_ENGINE_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD_UART_msg_parser
    {
        enum
        {
            FD_UART_PREAMBLE1 = 0,
            FD_UART_PREAMBLE_ID,
            FD_UART_DATA
        } msg_state;

        uint16_t length;
        uint16_t read_idx;
        uint8_t data[FD_MSG_ENGINE_LEN];
    } _msg;

    FD_engine();
    
    /* Do not allow copies */
    FD_engine(const FD_engine &other) = delete;
    FD_engine &operator=(const FD_engine&) = delete;

    static const uint8_t PREAMBLE1 = 0xFF;
    uint8_t PREAMBLE_ID = 0x0;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void cal_sumcheck() override {};
    void swap_message() override {};

    FD_UART_MSG_1 _msg_1;
};
