#include "FD2_message.h"

#define FD2_MSG2APM_UE4_GIMBAL_LEN 17
class FD2_msg2apm_ue4_gimbal : public FD2_message{
public:
    struct PACKED FD2_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD2_msg_header header;
        uint8_t  vehicle_id;
        float    roll;   // degree
        float    pitch;  // degree
        float    yaw;    // degree
        uint8_t  find_target;
        uint8_t  sum_check;        //byte 3 to 16
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD2_MSG2APM_UE4_GIMBAL_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure 
    struct PACKED FD2UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD2_MSG2APM_UE4_GIMBAL_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD2UART_msg_parser
    {
        enum
        {
            FD2UART_PREAMBLE1 = 0,
            FD2UART_PREAMBLE2,
            FD2UART_DATA,
            FD2UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD2_MSG2APM_UE4_GIMBAL_LEN];
    } _msg;

    FD2_msg2apm_ue4_gimbal();
    
    /* Do not allow copies */
    FD2_msg2apm_ue4_gimbal(const FD2_msg2apm_ue4_gimbal &other) = delete;
    FD2_msg2apm_ue4_gimbal &operator=(const FD2_msg2apm_ue4_gimbal&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x66;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD2UART_MSG_1 _msg_1;
};
