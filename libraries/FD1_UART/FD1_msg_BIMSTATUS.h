#include "FD1_message.h"

#define FD1_MSG_BIMSTATUS_LEN 150
class FD1_msg_BIMSTATUS : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;          // 0   2   无   EBH 90H
        uint16_t length;                // 2   2   无   150
        uint8_t  idx;                   // 4   1   无   0x01
        uint8_t  version;               // 5   1   无   0x00
        uint8_t  flag_sim;              // 6   1   无   0：真实，1：仿真
        uint16_t uav_type;              // 7   2   无   1：无人车；2：旋翼；3：固定翼；4：非合作目标
        uint32_t uav_id;                // 9   4   无   1~4294967295
        float    lng;                   // 13  4   有   Y=X*180/（231-1），单位：°
        float    lat;                   // 17  4   有   Y=X*180/（231-1），单位：°
        int16_t  alt_baro;              // 21  2   有   Y=X，单位：m，有符号
        uint16_t nouse_1;               // 23  2   无   
        uint16_t nouse_2;               // 25  2   无   
        uint16_t nouse_3;               // 27  2   无   
        uint16_t nouse_4;               // 29  2   无   
        uint16_t nouse_5;               // 31  2   无   
        uint16_t nouse_6;               // 33  2   无   
        uint16_t nouse_7;               // 35  2   无   
        int16_t  pitch;                 // 37  2   有   Y=X*180/（215-1），单位：°
        int16_t  roll;                  // 39  2   有   Y=X*180/（215-1），单位：°
        int16_t  yaw;                   // 41  2   无   Y=X*360/（216-1），单位：°，以正北（Y轴正向）为0°，顺时针为正，0~360°
        uint16_t nouse_8;               // 43  2   无   
        uint16_t power_rest;            // 45  2   无   电量，Y=X，单位：%
        int32_t  dist_roll;             // 47  4   有   Y=X/10，单位m，指向机头方向左负右正
        uint8_t  nouse_8;               // 51  1   无   
        uint8_t  nouse_9;               // 52  1   无    
        uint16_t target_speed;          // 53  2   无   Y=X/16，单位：km/h
        int16_t  target_alt;            // 55  2   有   Y=X，单位：m
        uint8_t  nouse_10;              // 57  1   无   
        uint16_t next_wp_id;            // 58  2   无   Y=X，1~65535
        uint32_t next_wp_dist;          // 60  4   无   Y=X，单位：m
        uint16_t nouse_11;              // 64  2   无   
        uint8_t  plat_switch_cmd;       // 66  1   无   
        uint8_t  plat_switch_act;       // 67  1   无   0：未执行，1：已执行
        uint8_t  plat_input_cmd;        // 68  1   无   
        uint8_t  plat_input_param[28];  // 69  28  无   
        uint8_t  plat_input_act;        // 97  1   无   0：未执行，1：已执行
        uint8_t  nouse_12;              // 98  1   无   
        uint8_t  nouse_13;              // 99  1   无   
        uint8_t  nouse_14;              // 100 1   无   
        uint8_t  nouse_15;              // 101 1   无   
        uint8_t  nouse_16;              // 102 1   无   
        uint8_t  nouse_17[28];          // 103 28  无   
        uint8_t  nouse_18;              // 131 1   无   
        int32_t  pos_x;                 // 132 4   有   Y=X/100，单位：m
        int32_t  pos_y;                 // 136 4   有   Y=X/100，单位：m
        int32_t  pos_z;                 // 140 4   有   Y=X/100，单位：m
        uint8_t  control_mode;          // 144 1   无   0：人工；1：系统自动；2：系统人工
        uint8_t  uav_moving_status;     // 145 1   无   0：空闲；1：移动
        uint8_t  arm_status;            // 146 1   无   0：否；1：是
        uint16_t copter_speed;          // 147 2   无   Y=X/100，单位：m/s，范围0~10m/s
        uint8_t  sum;                   // 149 1   无   相加取低八位
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_BIMSTATUS_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_BIMSTATUS_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_ID,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_BIMSTATUS_LEN];
    } _msg;

    FD1_msg_BIMSTATUS();
    
    /* Do not allow copies */
    FD1_msg_BIMSTATUS(const FD1_msg_BIMSTATUS &other) = delete;
    FD1_msg_BIMSTATUS &operator=(const FD1_msg_BIMSTATUS&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override; 

    FD1UART_MSG_1 _msg_1;
};
