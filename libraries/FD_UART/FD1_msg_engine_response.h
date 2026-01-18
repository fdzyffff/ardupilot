#include "FD1_message.h"

#define FD1_MSG_ENGINE_RESPONSE_LEN 85
class FD1_msg_engine_response : public FD1_message{
public:
    struct PACKED MSG_Collection {
        uint16_t size;
        uint8_t flag;
        uint16_t seconds;
        uint16_t pulsewidth1;
        uint16_t pulsewidth2;
        uint16_t rpm;
        uint16_t advance;
        uint16_t barometer;
        uint16_t map;
        uint16_t mat;
        uint16_t coolant;
        uint16_t tps;
        uint16_t batteryvoltage;
        uint16_t afr1;
        uint16_t afr2;
        uint16_t barocorrection;
        uint16_t gammaenrich;
        uint16_t ve1;
        uint16_t cold_adv_deg;
        uint16_t tpsdot;
        uint16_t mapdot;
        uint16_t egov1;
        uint16_t egov2;
        uint32_t crc32;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_ENGINE_RESPONSE_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length = FD1_MSG_ENGINE_RESPONSE_LEN;
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
        uint8_t data[FD1_MSG_ENGINE_RESPONSE_LEN];
    } _msg;

    FD1_msg_engine_response();
    
    /* Do not allow copies */
    FD1_msg_engine_response(const FD1_msg_engine_response &other) = delete;
    FD1_msg_engine_response &operator=(const FD1_msg_engine_response&) = delete;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    uint32_t _last_byte_ms;
    uint8_t _msg_crc_count;

    const float SF_INT16 = 65536.f/360.f;
};
