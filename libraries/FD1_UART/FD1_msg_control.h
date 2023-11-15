#include "FD1_message.h"

#define FD1_MSG_CONTROL_LEN 28
class FD1_msg_control : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header; //2
        uint8_t device_class;
        uint8_t device_type;
        uint8_t device_id;
        uint8_t gcs_class;
        uint8_t gcs_id;
        uint8_t frame_id;
        uint8_t cmd1;
        uint8_t cmd2;         //8
        float content2;
        int16_t ctrl_pitch;
        int16_t ctrl_roll;
        int16_t ctrl_yaw;
        uint8_t brake_left;
        uint8_t brake_right;
        uint8_t throttle_type;
        uint16_t ctrl_throttle;
        uint8_t count;         //1
        uint16_t sum_check;    //2
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_CONTROL_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint8_t length = FD1_MSG_CONTROL_LEN;
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

        uint16_t read;
        const uint8_t length = FD1_MSG_CONTROL_LEN;
        uint8_t count;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_CONTROL_LEN];
    } _msg;

    FD1_msg_control();
    
    /* Do not allow copies */
    FD1_msg_control(const FD1_msg_control &other) = delete;
    FD1_msg_control &operator=(const FD1_msg_control&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t FRAMETYPE = 0x01;
    static const uint8_t FRAMEID = 0x01;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool sum_check();

    FD1UART_MSG_1 _msg_1;
};
