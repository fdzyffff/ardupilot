#include "FD1_message.h"

#define FD1_MSG_INIT_LEN 126
class FD1_msg_init : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message status
    struct PACKED MSG_Status_1 {
        float lng;
        float lat;
        float alt;
        uint16_t speed;
    };

    // message param
    struct PACKED MSG_Param_1 {
        float lng;
        float lat;
        float alt;
        float pitch;
        float roll;
        float yaw;
        float speed;
        float speed_y;
        float speed_x;
        float fuel;
        uint8_t takeoff_type;
        uint8_t role;
        uint8_t status;
        uint8_t number;
        MSG_Status_1 status[5]; //70
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
        uint8_t empty1;        //7
        MSG_Param_1 param;     //114
        uint8_t count;         //1
        uint16_t sum_check;    //2
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_INIT_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint8_t length;
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
        const uint8_t length = FD1_MSG_INIT_LEN;
        uint8_t count;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_INIT_LEN];
    } _msg;

    FD1_msg_init();
    
    /* Do not allow copies */
    FD1_msg_init(const FD1_msg_init &other) = delete;
    FD1_msg_init &operator=(const FD1_msg_init&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t FRAMETYPE = 0x11;
    static const uint8_t FRAMEID = 0xB1;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool sum_check();

    FD1UART_MSG_1 _msg_1;
};
