#include "FD1_message.h"

#define FD1_MSG_LS_CONTROL_LEN 50
class FD1_msg_LS_control : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t data[44];
        uint8_t sum;
    };
    
    struct PACKED MSG_Collection_0x1A {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t rev;
        float target_throttle;
        float target_pitch;
        float target_roll;
        float flight_status;
        uint8_t reserved[30];
        uint8_t sum;
    };
    
    struct PACKED MSG_Collection_0xFD {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t rev;
        float target_speed;
        float target_pitch_rate;
        float target_yaw_rate;
        float flight_status;
        uint8_t reserved[30];
        uint8_t sum;
    };
    
    struct PACKED MSG_Collection_0xFE {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t rev;
        float cmd_vel_x;
        float cmd_vel_y;
        float cmd_vel_z;
        float flight_status;
        uint8_t reserved[30];
        uint8_t sum;
    };


    struct PACKED MSG_Collection_0x3C {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t rev;
        float target_speed;
        float target_alt;
        float target_roll;
        float flight_status;
        uint8_t reserved[30];
        uint8_t sum;
    };
    
    struct PACKED MSG_Collection_0x55 {
        FD1_msg_header header;
        uint8_t type;
        uint16_t length;
        uint8_t rev;
        double wp_lng;
        double wp_lat;
        float wp_alt;
        float target_speed;
        float flight_status;
        uint8_t reserved[18];
        uint8_t sum;
    };
    
    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        MSG_Collection_0x1A msg_0x1A;
        MSG_Collection_0xFD msg_0xFD;
        MSG_Collection_0xFE msg_0xFE;
        MSG_Collection_0x3C msg_0x3C;
        MSG_Collection_0x55 msg_0x55;
        uint8_t data[FD1_MSG_LS_CONTROL_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_LS_CONTROL_LEN;
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
        uint8_t data[FD1_MSG_LS_CONTROL_LEN];
    } _msg;

    FD1_msg_LS_control();
    
    /* Do not allow copies */
    FD1_msg_LS_control(const FD1_msg_LS_control &other) = delete;
    FD1_msg_LS_control &operator=(const FD1_msg_LS_control&) = delete;

    static const uint8_t PREAMBLE1 = 0xBE;
    static const uint8_t PREAMBLE2 = 0xBE;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
