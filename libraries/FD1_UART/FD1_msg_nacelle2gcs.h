#include "FD1_message.h"

#define FD1_MSG_NACELLE2gcs2GCS_LEN 55
class FD1_msg_nacelle2gcs : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t head_3;
        uint8_t info;
        uint8_t id;
    };

    struct PACKED MSG_X {
        uint8_t name;
        uint8_t class_number;
        uint16_t serial_number;
        uint16_t yaw_version;
        uint16_t pitch_version;
        uint8_t empty[18];
    };

    struct PACKED MSG_M {
        uint8_t type;
        uint8_t empty[7];
        int16_t roll_angle;
        int16_t pitch_angle;
        int16_t yaw_angle;
        uint8_t date[2];
        uint8_t time[3];
        int16_t gps_heading;
        uint8_t empty1;
        int32_t latitude;
        int32_t longitude;
        int32_t alt;
        int16_t gps_vel_xy;
        int16_t gps_hdop;
        int16_t gps_vdop;
        int16_t gps_vel_z;
    };


    union PACKED MSG_Collection {
        MSG_X msg_x;
        MSG_M msg_m;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        MSG_Collection sub_msg; 
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_NACELLE2gcs2GCS_LEN];
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
            FD1UART_LENGTH,
            FD1UART_DATA,
            FD1UART_POSTAMBLE1,
            FD1UART_POSTAMBLE2
        } msg_state;

        uint16_t read_idx;
        uint8_t length;
        uint16_t count;
        uint8_t data[FD1_MSG_NACELLE2gcs2GCS_LEN];
    } _msg;

    FD1_msg_nacelle2gcs();
    
    /* Do not allow copies */
    FD1_msg_nacelle2gcs(const FD1_msg_nacelle2gcs &other) = delete;
    FD1_msg_nacelle2gcs &operator=(const FD1_msg_nacelle2gcs&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;
    static const uint8_t POSTAMBLE1 = 0xAA;
    static const uint8_t POSTAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void reset(uint8_t temp) override;
    void reset2(uint8_t temp);
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
