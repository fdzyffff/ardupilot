#include "FD1_message.h"

#define FD1_MSG_HIL_IN_LEN 30
class FD1_msg_hil_in : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        int16_t scene_mode;
        int16_t ctrl_mode;
        int16_t landing_gear;
        int16_t ctrl_pitch_cd;
        int16_t ctrl_roll_cd;
        int16_t ctrl_x_vel_cms;
        int16_t ctrl_y_vel_cms;
        int16_t ctrl_z_vel_cms;
        int16_t ctrl_yaw_cd;
        int16_t ctrl_yaw_rate_crads;
        int16_t angle_yaw_cd;
        int16_t vel_x_cms;
        int16_t vel_y_cms;
        uint8_t sum_check;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_HIL_IN_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_HIL_IN_LEN;
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
            FD1UART_END,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_HIL_IN_LEN];
    } _msg;

    FD1_msg_hil_in();
    
    /* Do not allow copies */
    FD1_msg_hil_in(const FD1_msg_hil_in &other) = delete;
    FD1_msg_hil_in &operator=(const FD1_msg_hil_in&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x92;
    static const uint8_t POSTAMBLE = 0xFF;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
