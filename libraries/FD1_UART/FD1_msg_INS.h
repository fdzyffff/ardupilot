#include "FD1_message.h"

#define FD1_MSG_INS_LEN 118
class FD1_msg_INS : public FD1_message{
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
        uint32_t start_ms;
        uint32_t work_ms;
        uint8_t state;
        uint8_t nav_state;
        uint32_t error_code;
        int32_t pitch_micro_deg;
        int32_t roll_micro_deg;
        int32_t yaw_micro_deg;
        int32_t lng;
        int32_t lat;
        int32_t alt_mm;
        int32_t vel_e; //1E-4
        int32_t vel_n;
        int32_t vel_u; //54
        float rate_e_degrees;
        float rate_n_degrees;
        float rate_u_degrees;
        float acc_x_mss;
        float acc_y_mss;
        float acc_z_mss; //78
        uint8_t gps_ok;
        uint32_t gps_pps;
        uint32_t gps_utc;
        int32_t gps_lng;
        int32_t gps_lat;
        int32_t gps_alt_mm;
        int32_t gps_vel_e_ms_o4; // m/s 1E-4
        int32_t gps_vel_n_ms_o4; // m/s 1E-4
        int16_t gps_vel_u_ms_o2; // m/s 1E-2
        uint8_t gps_fix_state; //GGA
        uint8_t gps_numstat;
        int16_t gps_height_error; // m 1E-2
        uint16_t gps_hdop; //1E-2
        uint16_t gps_vdop; //1E-2
        uint8_t xor_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_INS_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_INS_LEN;
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

        uint16_t length;
        uint16_t read;
        uint8_t xor_check;
        uint8_t data[FD1_MSG_INS_LEN];
    } _msg;

    FD1_msg_INS();
    
    /* Do not allow copies */
    FD1_msg_INS(const FD1_msg_INS &other) = delete;
    FD1_msg_INS &operator=(const FD1_msg_INS&) = delete;

    static const uint8_t PREAMBLE1 = 0x55;
    static const uint8_t PREAMBLE2 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
