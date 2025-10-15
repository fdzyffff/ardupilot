#include "FD_QD_message.h"

#define FD_MSG_QD_S11_LEN 73
class FD_msg_QD_S11 : public FD_QD_message{
public:
    struct PACKED FD_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // QD/QP/QG 系列：当遥测状态帧2 未使能时，遥测数据频率为50hz，帧1 刷新率为50hz；当遥测状态帧2 使能时，遥测数据频率为50hz，帧1 刷新率为25hz，帧2 刷新率为25hz，交替返回；默认为未使能状态。
    // QEC 系列：当接入吊舱设备时，遥测数据频率为50hz，帧1 刷新率为25hz，帧2 刷新率为25hz，交替返回；当未接入吊舱设备时，遥测数据频率为50hz，帧1 刷新率为0hz，帧2 刷新率为50hz。

    // message structure
    struct PACKED MSG_Command_1 {
        FD_msg_header header;// 1 帧头1 UINT8 0xAA // 2 帧头2 UINT8 0x55
        uint8_t frame_length;  // 3 帧长UINT8 整帧数据长度
        uint8_t frame_ID; // 4 帧识别码UINT8 0x11：系统状态帧
        uint8_t system_mode; // 5 系统模式UINT8 详细见C1#M 包定义
        uint8_t tune_status;  // 6 调测状态UINT8 Bit0：漂移补偿0 关，1 开 Bit1：LMC 0 关，1 开 Bit2：微调指令0 无, 1 成功 Bit3：校轴0 关, 1 开 Bit4：校靶指令0 无, 1 成功 Bit5~Bit7：保留
        uint8_t servo_error_code; // 7 伺服故障代码UINT8 Bit0：平台自检0 无故障，1 故障 Bit1：方位编码器0 无故障，1 故障 Bit2：俯仰编码器0 无故障，1 故障 Bit3：滚转编码器0 无故障，1 故障 Bit4：陀螺仪0 无故障，1 故障 Bit5：平台通信0 无故障，1 故障 Bit6：外方位编码器0 无故障，1 故障 （两轴四框架） Bit7：外俯仰编码器0 无故障，1 故障（两轴四框架）
        uint8_t payload_status; // 8 载荷状态UINT8 Bit0：可见光传感器1，0 无故障，1 故障 Bit1：可见光传感器2，0 无故障，1 故障 Bit2：红外传感器， 0 无故障，1 故障 Bit3：激光传感器， 0 无故障，1 故障 Bit4：可见光传感器1 电源，0 未上电，1 已上电 Bit5：可见光传感器2 电源，0 未上电，1 已上电 Bit6：红外传感器电源，0 未上电，1 已上电 Bit7：激光传感器电源，0 未上电，1 已上电
        int16_t cam_yaw; // 9 吊舱方位角INT16 LSB=360/65536°，-180°～180°
        int16_t cam_pitch; // 11 吊舱俯仰角INT16 LSB=360/65536°，-120°～+90°
        int16_t cam_roll; // 13 吊舱滚转角INT16 LSB=360/65536°，-180°～180°
        int16_t cam_yaw_rate; // 15 方位角速度INT16 LSB=0.01°/s
        int16_t cam_pitch_rate; // 17 俯仰角速度INT16 LSB=0.01°/s
        int16_t cam_roll_rate; // 19 滚转角速度INT16 LSB=0.01°/s
        int16_t cam_att_pitch; // 21 吊舱俯仰姿态角INT16 LSB=360/65536°，-90°～90°，水平为零，上为正，下为负；
        int16_t cam_att_roll; // 23 吊舱滚转姿态角INT16 LSB=360/65536°，-180°～180°，水平为零，右倾为正，左倾为负
        int16_t cam_att_yaw; // 25 吊舱方位姿态角INT16 LSB=360/65536°，-180°～180°，正北为零，北偏东为正，北偏西为负
        int32_t target_lng;// 27 地理定位目标经度INT32 LSB=10^-7°
        int32_t target_lat; // 31 地理定位目标纬度INT32 LSB=10^-7°
        int16_t target_alt; // 35 地理定位目标高度INT16 LSB=1m
        uint8_t leaser_status; // 37 激光状态UINT8 Bit0：使能状态； 0x00：未使能； 0x01：已使能；Bit1~3：工作模式； 0x00：停止； 0x01：单次测距； 0x02：1hz 连续测距； 0x03：5hz 连续测距； 0x04：激光照射； Bit4~7：激光编码； 0x00：无效； 0x01：编码1； ...
        uint8_t error_code; // 38 UINT8 故障码，定义以实际为准；
        uint16_t leaser_distance;// 39 UINT16 距离值；LSB=1m；
        uint16_t leaser_status_2; // 41 激光补充状态UINT16 Bit0~3：距离值小数位，LSB=0.1m； Bit4：测距有效标志； 0x00：无效 0x01：有效 Bit5：测距计数； Bit6~9：测距延迟补偿，LSB=10ms； Bit10~15：备用；
        uint8_t track_status; // 43 跟踪状态UINT8 Bit0~1：跟踪状态； 0x00：停止； 0x01：丢失； 0x02：跟踪； 0x03：预跟踪； Bit2~4：模板大小状态； 0x01：16*16； 0x02：32*32； 0x03：64*64； 0x04：128*128； Bit5~7：备用
        uint8_t target_x; // 44 UINT16 目标中心水平坐标； LSB=1pixel，左上为零点，右下为正；
        uint8_t target_y; // 46 UINT16 目标中心垂直坐标； LSB=1pixel，左上为零点，右下为正；
        uint8_t target_width; // 48 UINT16 目标宽度；LSB=1pixel
        uint8_t target_height; // 50 UINT16 目标高度；LSB=1pixel
        uint8_t unused_1; // 52 备用
        uint8_t unused_2; // 54 当前主图像传感器UINT8 Bit0~3：传感器通道 0x00：可见光传感器1；状态0x01：可见光传感器2； 0x02：红外传感器1； 0x03：红外传感器2； Bit4~7：电子变倍状态； 0：X1（关）； 1：X2； ... 15：X16；
        uint8_t unused_3; // 55 UINT8 Bit0~1：图像增强状态； 0x00：增强关； 0x01：增强1 档； 0x02：增强2 档； 0x03：增强3 档； Bit2：透雾状态； 0x00：透雾关； 0x01：透雾开； Bit3：近红外状态； 0x00：近红外关； 0x01：近红外开； Bit4：稳像状态； 0x00：稳像关； 0x01：稳像开； Bit5~6：视场角缩放状态； 0x00：停止变化； 0x01：视场放大中； 0x02：视场变小中； Bit7：伪彩模式； 0x00：伪彩关； 0x01：伪彩开；
        uint16_t cam_zoom; // 56 UINT16 视场角放大倍数； LSB=0.1 倍；
        uint16_t cam_x_fov;// 58 UINT16 水平视场角； LSB=0.01°；
        uint16_t cam_y_fov; // 60 UINT16 垂直视场角； LSB=0.01°；
        uint8_t unused[10];
         // 62 图像状态UINT8 Bit0：电子稳像开关； 0x00：关； 0x01：开；Bit1：画中画开关； 0x00：关； 0x01：开； Bit2：目标检测开关； 0x00：关；0x01：开； Bit3~7：备用；
         // 63 备用UINT8 -
         // 64 编码状态（多传感器时交替回传）UINT8 Bit0~3：传感器通道 0x00：可见光传感器1； 0x01：可见光传感器2； 0x02：红外传感器1； 0x03：红外传感器2； Bit4~6：视频分辨率； 1：640*480/480P； 2：1280*720/720P； 3：1920*1080/1080P； 4：2048*1152/2K； 5：4096*2304/4K； Bit7：编码格式； 0x00：H264； 0x01：H265；
         // 65 UINT8 Bit0~6：拍照计数值，lSB=1 Bit7：录像状态 0x00：关； 0x01：开；
         // 66 UINT16 Bit0~14 存储容量，LSB=0.1GB
         // 67 Bit15：容量标志； 0x00：剩余容量； 0x01：总容量；
         // 68 UINT16 视频码流，LSB=1kbps
         // 70 UINT8 视频帧率，LSB=1fps
         // 71 UINT8 GOP 值，LSB=1
         // 72 系统温度INT8 LSB=1℃，-128～127
        uint8_t sum_check; // 73 校验字UINT8 前面所有字节求和取低八位
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_QD_S11_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_MSG_QD_S11_LEN;
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
        uint8_t sum_check;
        uint8_t data[FD_MSG_QD_S11_LEN];
    } _msg;

    FD_msg_QD_S11();
    
    /* Do not allow copies */
    FD_msg_QD_S11(const FD_msg_QD_S11 &other) = delete;
    FD_msg_QD_S11 &operator=(const FD_msg_QD_S11&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check();

    FD1UART_MSG_1 _msg_1;
};
