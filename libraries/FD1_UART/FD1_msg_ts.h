#include "FD1_message.h"

#define FD1_MSG_TS_LEN 64
class FD1_msg_ts : public FD1_message{
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

    struct PACKED MSG_sub_T1 {
        uint8_t empty[2];
        int32_t plane_lat;
        int32_t plane_lng;
        int16_t plane_alt;
        int32_t target_lat;
        int32_t target_lng;
        int16_t target_alt;
    };

    struct PACKED MSG_sub_F1 {
        uint8_t empty[1];
    };

    struct PACKED MSG_sub_B1 {
        uint8_t empty[6];
    };

    struct PACKED MSG_sub_D1 {
        uint8_t empty[12];
    };

    struct PACKED MSG_sub_K1 {
        uint8_t ret[5];
        uint8_t status;
        uint8_t target_ret;
        uint16_t target_x1;
        uint16_t target_y1;
        uint16_t target_x2;
        uint16_t target_y2;
        int16_t confidence_level;
    };

    struct PACKED MSG_40 {
        MSG_sub_T1 sub_t1;
        MSG_sub_F1 sub_f1;
        MSG_sub_B1 sub_b1;
        MSG_sub_D1 sub_d1;
        MSG_sub_K1 sub_k1;
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
        MSG_X  msg_x;
        MSG_M  msg_m;
        MSG_40 msg_40;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        MSG_Collection sub_msg; 
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_TS_LEN];
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
            FD1UART_PREAMBLE3,
            FD1UART_INFO,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        uint8_t length;
        uint8_t count;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_TS_LEN];
    } _msg;

    FD1_msg_ts();
    
    /* Do not allow copies */
    FD1_msg_ts(const FD1_msg_ts &other) = delete;
    FD1_msg_ts &operator=(const FD1_msg_ts&) = delete;

    static const uint8_t PREAMBLE1 = 0x55;
    static const uint8_t PREAMBLE2 = 0xAA;
    static const uint8_t PREAMBLE3 = 0xDC;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool sum_check();
    bool set_id(uint8_t id);

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
