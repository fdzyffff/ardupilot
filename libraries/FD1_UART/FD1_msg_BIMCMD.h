#include "FD1_message.h"

#define FD1_MSG_BIMCMD_LEN 100
class FD1_msg_BIMCMD : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // 旋翼机航点装订
    struct PACKED MSG_Input_56H {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        uint16_t wp_idx;        // 21~22      航点号 　        1-65535               1             1                Y=X
        float    wp_lng;        // 23~26      航点经度    　   -180°-180°             0.0000001°   180/(2^31- 1)    Y=X*180/(2^31 - 1)
        float    wp_lat;        // 27~30      航点纬度    　   -90°-90°               0.0000001°   180/(2^31- 1)    Y=X*180/(2^31 - 1)
        int16_t  wp_alt;        // 31~32      航点海拔高度     -500m-15000m           1m            1               Y=X
        int32_t  wp_pos_x;      // 33~36      X   　          -10000000m-10000000m   0.01m         1/100           Y=X/100
        int32_t  wp_pos_y;      // 37~40      Y   　          -10000000m-10000000m   0.01m         1/100           Y=X/100
        int32_t  wp_pos_z;      // 41~44      Z   　          -10000000m-10000000m   0.01m         1/100           Y=X/100
        uint8_t  wp_type;       // 45         航点类型         0：平飞点   1：起飞点   2：着陆点
        uint16_t wp_speed;      // 46~47      航点速度    　   0~10m/s                0.01m/s       1               Y=X/100
        uint8_t  empty;         // 48         空                   
    };

    // 旋翼机向点飞行
    struct PACKED MSG_Input_72H {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        float    target_pos_x;   //21~24      X              -10000000m-10000000m    0.01m         1/100           Y=X/100
        float    target_pos_y;   //25~28      Y              -10000000m-10000000m    0.01m         1/100           Y=X/100
        float    target_pos_z;   //29~32      Z              -10000000m-10000000m    0.01m         1/100           Y=X/100
        uint8_t  empty[16];      //33~48      空                   
    };

    // 旋翼机速度给定
    struct PACKED MSG_Input_74H {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        uint16_t target_speed;   //21~22      给定速度        0~10m/s                 0.01m/s       1               Y=X/100                      
        uint8_t  empty[26];      //33~48      空                   
    };

    // 旋翼机高度给定
    struct PACKED MSG_Input_76H {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        int16_t  target_alt;     //21~22      给定高度        -500~20000m             1m            1               Y=X                    
        uint8_t  empty[26];      //33~48      空                   
    };

    // 旋翼机航向给定
    struct PACKED MSG_Input_78H {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        uint16_t target_yaw;     //21~22      给定航向         0°~360°                0.1°         360/(2^16-1)     Y=X*360/(2^16-1)
        uint8_t  empty[26];      //33~48      空                   
    };

    // 旋翼机XYZ位移
    struct PACKED MSG_Input_7AH {//遥控帧字节  信号名          参数范围                精度          分辨率           变换关系
        float    offset_pos_x;   //21~24      X              -10000000m-10000000m    0.01m         1/100           Y=X/100
        float    offset_pos_y;   //25~28      Y              -10000000m-10000000m    0.01m         1/100           Y=X/100
        float    offset_pos_z;   //29~32      Z              -10000000m-10000000m    0.01m         1/100           Y=X/100
        uint8_t  empty[12];      //33~48      空                   
    };

    union PACKED Content_Input {
        MSG_Input_56H input_56H;
        MSG_Input_72H input_72H;
        MSG_Input_74H input_74H;
        MSG_Input_76H input_76H;
        MSG_Input_78H input_78H;
        MSG_Input_7AH input_7AH;
        uint8_t data[28];
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;          // 0   2   无   EBH 90H
        uint16_t legth;                 // 2   2   100
        uint8_t  idx;                   // 4   1   0xF1: 多机通用遥控帧
        uint8_t  version;               // 5   1   0x00
        uint8_t  flag_sim;              // 6   1   0：真实，1：仿真
        uint16_t uav_type;              // 7   2   1：无人车；2：旋翼；3：固定翼；4：非合作目标
        uint32_t uav_id;                // 9   4   1~4294967295
        uint16_t gcs_id;                // 13  2   默认0
        uint8_t  plat_switch_cmd[3];    // 15  3   三判二
        uint8_t  plat_input_cmd[3];     // 18  3   三判二
        Content_Input plat_input_param; // 21  28  
        uint8_t  nouse_1;               // 49  1   
        uint8_t  nouse_2;               // 50  1   
        uint8_t  nouse_3;               // 51  1   
        uint8_t  nouse_4;               // 52  1   
        uint8_t  nouse_5;               // 53  1   
        uint8_t  nouse_6;               // 54  1   
        uint8_t  nouse_7;               // 55  1   
        uint8_t  nouse_8;               // 56  1   
        uint8_t  nouse_9[3];            // 57  3   
        uint8_t  nouse_10[3];           // 60  3   
        uint8_t  nouse_11[28];          // 63  28  
        uint16_t  nouse_12;             // 91  2   
        uint16_t  nouse_13;             // 93  2   
        uint16_t  nouse_14;             // 95  2   
        uint16_t  nouse_15;             // 97  2
        uint8_t  sum;                   // 99  1   相加取低八位
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_BIMCMD_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_BIMCMD_LEN;
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
        uint8_t data[FD1_MSG_BIMCMD_LEN];
    } _msg;

    FD1_msg_BIMCMD();
    
    /* Do not allow copies */
    FD1_msg_BIMCMD(const FD1_msg_BIMCMD &other) = delete;
    FD1_msg_BIMCMD &operator=(const FD1_msg_BIMCMD&) = delete;

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override; 

    FD1UART_MSG_1 _msg_1;
};
