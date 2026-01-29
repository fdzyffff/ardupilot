#include "FD1_message.h"

#define FD1_MSG_0XA1_LEN 60
class FD1_msg_0XA1 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED FD1_msg_calibration {
        float direction_x;
        float direction_y;
        float direction_z;
        uint8_t compass_id;
        uint8_t cal_mask;
        uint8_t cal_status;
        uint8_t attempt;
        uint8_t completion_pct;
        uint8_t completion_mask1;
        uint8_t completion_mask2;
        uint8_t completion_mask3;
        uint8_t completion_mask4;
        uint8_t completion_mask5;
        uint8_t completion_mask6;
        uint8_t completion_mask7;
        uint8_t completion_mask8;
        uint8_t completion_mask9;
        uint8_t completion_mask10;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t length;
        uint8_t ID;
        FD1_msg_calibration mag_cal[2];
        uint16_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_0XA1_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length;
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
            FD1UART_SUM1,
            FD1UART_SUM2,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_0XA1_LEN];
    } _msg;

    FD1_msg_0XA1();
    
    /* Do not allow copies */
    FD1_msg_0XA1(const FD1_msg_0XA1 &other) = delete;
    FD1_msg_0XA1 &operator=(const FD1_msg_0XA1&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
