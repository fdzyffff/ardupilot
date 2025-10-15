#include "FD_QD_message.h"

#define FD_MSG_QD_S12_LEN 100
class FD_msg_QD_S12 : public FD_QD_message{
public:
    struct PACKED FD_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD_msg_header header;// 1 帧头1 UINT8 0xAA // 2 帧头2 UINT8 0x55
// 1 帧头1 UINT8 0xAA
// 2 帧头2 UINT8 0x55
// 3 帧长UINT16 整帧数据长度
// 5 帧识别码UINT8 0x22，系统状态帧
// 6 图像帧ID UINT32 图像帧ID
// 10 时间戳UINT64 当前外部授时时间戳，LSB=1ms
// 18 目标数量UINT8 最大数目为32
// 19 目标1 类别UINT8 目标类别； 0x00：飞机； 0x01：船； 0x02：车； 0x03：人； ... 0xFF：模板匹配结果
// 20 ID UINT8 LSB=1
// 21 置信度UINT8 LSB=0.00390625
// 22 水平坐标 UINT16 LSB=1pixel，左上为零点，右下为正
// 24 垂直坐标 UINT16 LSB=1pixel，左上为零点，右下为正
// 26 宽度UINT16 LSB=1pixel
// 28 高度UINT16 LSB=1pixel
// ...
// 目标n
// 19+n*11 帧尾1 UINT8 0xAA
        uint8_t sum_check; // 73 校验字UINT8 前面所有字节求和取低八位
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD_MSG_QD_S12_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD_MSG_QD_S12_LEN;
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
        uint8_t data[FD_MSG_QD_S12_LEN];
    } _msg;

    FD_msg_QD_S12();
    
    /* Do not allow copies */
    FD_msg_QD_S12(const FD_msg_QD_S12 &other) = delete;
    FD_msg_QD_S12 &operator=(const FD_msg_QD_S12&) = delete;

    static const uint8_t PREAMBLE1 = 0xAA;
    static const uint8_t PREAMBLE2 = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check();

    FD1UART_MSG_1 _msg_1;
};
