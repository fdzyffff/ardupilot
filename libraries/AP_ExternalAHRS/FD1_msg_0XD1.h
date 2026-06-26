#include "FD1_message.h"

#define FD1_MSG_0XD1_LEN 140
class FD1_msg_0XD1 : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t length;
        uint8_t ID;
        uint32_t counter;
        uint8_t state;
        float pitch;
        float roll;
        float yaw;
        float yaw_gps;
        float pitch_rate;
        float roll_rate;
        float yaw_rate;
        int32_t lon;
        int32_t lat;
        int32_t alt_baro;
        int32_t alt_gps;
        int32_t alt;
        float velocity_x;
        float velocity_y;
        float velocity_z;
        uint8_t velocity_air;
        uint8_t cam_frame_rate;
        uint8_t ins_frame_rate;
        uint8_t visual_connect;
        float accel_x;
        float accel_y;
        float accel_z;
        uint8_t satellite_num;
        uint16_t hdop;
        uint16_t vdop;
        uint8_t gps_status;
        uint8_t gps_hh;
        uint8_t gps_mm;
        uint8_t gps_ss;
        int8_t temperature;
        int16_t HDT;
        int16_t HDG_Dev;
        uint8_t redundancy;
        uint8_t GPS0_DT;
        uint8_t GPS1_DT;
        float GPS_vx;
        float GPS_vy;
        float GPS_vz;
        uint16_t gps_ms;
        uint8_t gps_day;
        uint16_t gps_week;
        uint8_t ahrs_state;
        float east;
        float north;
        float up;
        uint16_t std_dev;
        uint16_t std_dev_up;
        uint16_t Engine_RPM;
        uint16_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_0XD1_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_0XD1_LEN;
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
        uint8_t data[FD1_MSG_0XD1_LEN];
    } _msg;

    FD1_msg_0XD1();
    
    /* Do not allow copies */
    FD1_msg_0XD1(const FD1_msg_0XD1 &other) = delete;
    FD1_msg_0XD1 &operator=(const FD1_msg_0XD1&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
