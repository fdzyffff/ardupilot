#include "FD1_message.h"

#define FD1_MSG_CAM_IN_LEN 10
class FD1_msg_cam_in : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t cam_id;
        int32_t cam_x_in;
        int32_t cam_y_in;
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_CAM_IN_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_CAM_IN_LEN;
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

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_CAM_IN_LEN];
    } _msg;

    FD1_msg_cam_in();
    
    /* Do not allow copies */
    FD1_msg_cam_in(const FD1_msg_cam_in &other) = delete;
    FD1_msg_cam_in &operator=(const FD1_msg_cam_in&) = delete;

    static const uint8_t PREAMBLE1 = 0xFB;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
