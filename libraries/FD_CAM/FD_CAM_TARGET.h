#include "FD1_message.h"

#define FD1_MSG_GIMBAL2GCS_LEN 64
class FD1_msg_gimbal2gcs : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t type;
        uint8_t self_check;
        uint16_t gimbal_status_1;
        uint16_t gimbal_status_2;
        uint8_t servo_status;
        int16_t angle_yaw;     //centi-degree
        int16_t angle_pitch;
        int16_t angle_roll;
        uint8_t empty_1[3];
        uint8_t target_info;
        uint8_t empty_2[2];
        uint8_t tf_usage;
        uint8_t tf_space;
        uint16_t ir_focus;
        uint16_t cam_focus;
        uint16_t target_id;
        float target_lng;
        float target_lat;
        int16_t target_alt;
        int16_t soc_temp;  //0.1 degree
        int16_t rate_yaw;  //centi-degree/s
        int16_t rate_pitch;
        int16_t rate_roll;
        uint8_t image_type;
        uint8_t cmd_feedback;
        uint8_t attitude_feedback;
        int16_t target_x;
        int16_t target_y;
        int16_t target_width;
        int16_t target_height;
        int16_t target_pixel_yaw;
        int16_t target_pixel_pitch;
        uint8_t xorsum;
        uint8_t end;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_GIMBAL2GCS_LEN];
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
            FD1UART_INFO,
            FD1UART_DATA,
            FD1UART_SUM,
            FD1UART_END,
        } msg_state;

        uint16_t read;
        uint8_t length;
        uint8_t count;
        uint8_t xorsum;
        uint8_t data[FD1_MSG_GIMBAL2GCS_LEN];
    } _msg;

    FD1_msg_gimbal2gcs();
    
    /* Do not allow copies */
    FD1_msg_gimbal2gcs(const FD1_msg_gimbal2gcs &other) = delete;
    FD1_msg_gimbal2gcs &operator=(const FD1_msg_gimbal2gcs&) = delete;

    static const uint8_t PREAMBLE1 = 0xFC;
    static const uint8_t PREAMBLE2 = 0x2C;
    static const uint8_t POSTAMBLE1 = 0xF0;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
