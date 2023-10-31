#include "FD1_message.h"

#define FD1_MSG_GUIDE_LEN 55
class FD1_msg_guide : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message guide_data
    struct PACKED MSG_Guide_data {
        uint8_t guide_type;
        float platform_lng;
        float platform_lat;
        float platform_alt;
        float platform_pitch;
        float platform_roll;
        float platform_yaw;
        float platform_vx;
        float platform_vy;
        float platform_vz;
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
        uint8_t mis_id; 
        uint8_t wp_num;
        uint8_t training_type; //9
        MSG_Guide_data guide_data; //41
        uint8_t count;         //1
        uint16_t sum_check;    //2
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_GUIDE_LEN];
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
        const uint8_t length = FD1_MSG_GUIDE_LEN;
        uint8_t count;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_GUIDE_LEN];
    } _msg;

    FD1_msg_guide();
    
    /* Do not allow copies */
    FD1_msg_guide(const FD1_msg_guide &other) = delete;
    FD1_msg_guide &operator=(const FD1_msg_guide&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t FRAMETYPE = 0x11;
    static const uint8_t FRAMEID = 0xF1;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool sum_check();

    FD1UART_MSG_1 _msg_1;
};
