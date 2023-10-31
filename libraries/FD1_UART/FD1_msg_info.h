#include "FD1_message.h"

#define FD1_MSG_INFO_LEN 145
class FD1_msg_info : public FD1_message{
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
    struct PACKED MSG_Info_1 {
        double lng;
        double lat;
        float alt;
        float pitch;
        float roll;
        float yaw_rate;
        float ve;
        float vn;
        float vu;
        float airspeed;
        float gps_speed;
        float attack_angle;
        float side_slip_angle;
        float wind_speed;
        float wind_direction;
        float flap_left;
        float flap_right;
        float elevator_left;
        float elevator_right;
        float rudder_left;
        float rudder_right;
        int8_t flight_stage;
        int8_t flight_mode;
        int8_t landinggear_state;
        uint8_t refly_state;
        float fule;
        float rpm;
        float wp_distance; // 112
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
        uint8_t cmd1;
        uint8_t cmd2;          //8
        MSG_Info_1 info;       //112
        uint8_t empty[20];     //20
        uint8_t count;         //1
        uint16_t sum_check;    //2
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_INFO_LEN];
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
        const uint8_t length = FD1_MSG_INFO_LEN;
        uint8_t count;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_INFO_LEN];
    } _msg;

    FD1_msg_info();
    
    /* Do not allow copies */
    FD1_msg_info(const FD1_msg_info &other) = delete;
    FD1_msg_info &operator=(const FD1_msg_info&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;
    static const uint8_t FRAMETYPE = 0x11;
    static const uint8_t FRAMEID = 0x41;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    bool sum_check();

    FD1UART_MSG_1 _msg_1;
};
