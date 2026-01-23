#include "FD1_message.h"

#define FD1_MSG_LS_STATUS_LEN 110
class FD1_msg_LS_status : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        float run_time;
        float gimbal_pitch;
        float gimbal_yaw;
        float target_x;
        float target_y;
        double target_lng;
        double target_lat;
        float target_alt;
        double current_lng;
        double current_lat;
        float vel_n;
        float vel_e;
        float vel_d;
        float roll;
        float pitch;
        float yaw;
        float air_speed;
        float yaw_rate;
        float pos_x;
        float pos_y;
        float pos_z;
        float current_alt;
        uint8_t sum;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_LS_STATUS_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_LS_STATUS_LEN;
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
        uint16_t length;
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_LS_STATUS_LEN];
    } _msg;

    FD1_msg_LS_status();
    
    /* Do not allow copies */
    FD1_msg_LS_status(const FD1_msg_LS_status &other) = delete;
    FD1_msg_LS_status &operator=(const FD1_msg_LS_status&) = delete;

    static const uint8_t PREAMBLE1 = 0xBE;
    static const uint8_t PREAMBLE2 = 0xBE;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
