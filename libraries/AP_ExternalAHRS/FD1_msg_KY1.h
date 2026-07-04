#include "FD1_message.h"

#define FD1_MSG_KY1_LEN 22
class FD1_msg_KY1 : public FD1_message{
public:
    // message structure
    struct PACKED MSG_Command_1 {
        uint8_t header;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t angle_roll;
        int16_t angle_pitch;
        int16_t angle_yaw;
        uint8_t status;
        uint16_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_KY1_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_KY1_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        const uint16_t length = FD1_MSG_KY1_LEN;
        uint16_t read;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_KY1_LEN];
    } _msg;

    FD1_msg_KY1();
    
    /* Do not allow copies */
    FD1_msg_KY1(const FD1_msg_KY1 &other) = delete;
    FD1_msg_KY1 &operator=(const FD1_msg_KY1&) = delete;

    static const uint8_t PREAMBLE1 = 0x36;

    // Scale factors from KY-1 protocol
    const float GYRO_SCALE  = 0.0175f;   // deg/s per LSB
    const float ACCEL_SCALE = 0.000488f;  // g per LSB
    const float ANGLE_SCALE = 0.01f;      // deg per LSB

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;

    uint32_t _last_byte_ms;
};
