#include "FD1_message.h"

#define FD1_MSG_ENGINE_REQUEST_LEN 13
class FD1_msg_engine_request : public FD1_message{
public:
    
    struct PACKED MSG_Collection {
        uint16_t size;
        uint8_t data[7];
        int32_t crc32;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_ENGINE_REQUEST_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_ENGINE_REQUEST_LEN;
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
        uint8_t data[FD1_MSG_ENGINE_REQUEST_LEN];
    } _msg;

    FD1_msg_engine_request();
    
    /* Do not allow copies */
    FD1_msg_engine_request(const FD1_msg_engine_request &other) = delete;
    FD1_msg_engine_request &operator=(const FD1_msg_engine_request&) = delete;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
