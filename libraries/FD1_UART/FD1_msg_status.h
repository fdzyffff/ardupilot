#include "FD1_message.h"

#define FD1_MSG_STATUS_LEN 137
class FD1_msg_status : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t head_3;
    };
    
    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t number_1;
        uint8_t number_2;
        uint16_t length;
        uint8_t head_1;
        uint8_t head_2;
        uint8_t head_3;
        uint8_t type;
        uint16_t data_length;
        uint32_t system_time_s;
        uint16_t utc_year;
        uint8_t utc_month;
        uint8_t utc_day;
        uint8_t utc_hour;
        uint8_t utc_minute;
        uint8_t utc_second;
        uint8_t gps_count;
        uint8_t bd_count;
        uint8_t gps_ok;
        double lng_deg;
        double lat_deg;
        float alt_m;
        float pos_n_m;
        float pos_e_m;
        float pos_d_m;
        float vel_n_ms;
        float vel_e_ms;
        float vel_d_ms;
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        float roll_rate_degs;
        float pitch_rate_degs;
        float yaw_rate_degs;
        float acc_x_g;
        float acc_y_g;
        float acc_z_g;
        float airspeed;
        float groundspeed;
        float baro_alt_m;
        float batt_volt;
        float batt_current;
        uint16_t motor_rpm;
        uint8_t throttle;
        uint8_t control_mode;
        uint8_t flight_stage;
        uint8_t parachute;
        uint8_t radio;
        uint8_t sum1;
        uint16_t sum2;
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_STATUS_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        int32_t length = FD1_MSG_STATUS_LEN;
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
            FD1UART_DATA,
            FD1UART_SUM1,
            FD1UART_SUM21,
            FD1UART_SUM22,
        } msg_state;

        uint16_t read;
        int32_t length;
        uint8_t count;
        uint8_t sum1;
        uint16_t sum2;
        uint8_t data[FD1_MSG_STATUS_LEN];
    } _msg;

    FD1_msg_status();
    
    /* Do not allow copies */
    FD1_msg_status(const FD1_msg_status &other) = delete;
    FD1_msg_status &operator=(const FD1_msg_status&) = delete;

    static const uint8_t PREAMBLE1 = 0xFA;
    static const uint8_t PREAMBLE2 = 0xFA;
    static const uint8_t PREAMBLE3 = 0xC2;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    void make_sum();

    FD1UART_MSG_1 _msg_1;

    const float SF_INT16 = 65536.f/360.f;
};
