#include "FD1_message.h"

#define FD1_MSG_LS_UOM_LEN 203
#define FD1_MSG_LS_UOM_MSG_NUM 21
#define FD1_MSG_LS_UOM_MSG_LENGT_MAX 20
class FD1_msg_LS_UOM : public FD1_message{
public:
    
    struct PACKED MSG_Collection {
        uint8_t type;
        uint8_t version;
        uint8_t length;
        uint8_t all_msg_data[200];
    };

    // message structure
    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_LS_UOM_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        Content_1 content;
    };

    FD1_msg_LS_UOM();
    
    /* Do not allow copies */
    FD1_msg_LS_UOM(const FD1_msg_LS_UOM &other) = delete;
    FD1_msg_LS_UOM &operator=(const FD1_msg_LS_UOM&) = delete;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    //字节位数据标识位数据内容项序号是否必选名称
    //第1字节
    //0x80 001 M 唯一产品识别码
    //0x40 002 M 实名登记标志
    //0x20 003 O 民用无人驾驶航空器系统运行类别
    //0x10 004 M 民用无人驾驶航空器分类
    //0x08 005 M 民用无人驾驶航空器遥控站位置类型
    //0x04 006 M 民用无人驾驶航空器遥控站位置
    //0x02 007 M 民用无人驾驶航空器遥控站高度
    //0x01       扩展标志位
    //
    //第2字节
    //0x80 008 M 民用无人驾驶航空器位置
    //0x40 009 M 航迹角
    //0x20 010 M 地速
    //0x10 011 O 相对高度
    //0x08 012 O 垂直速度
    //0x04 013 M 大地高度
    //0x02 014 O 气压高度
    //0x01       扩展标志位
    //
    //第3字节
    //0x80 015 M 运行状态
    //0x40 016 M 坐标系类型
    //0x20 017 M 水平精度
    //0x10 018 M 垂直精度
    //0x08 019 M 速度精度
    //0x04 020 M 时间戳
    //0x02 021 M 时间戳精度
    //0x01       扩展标志位

    void make_init();
    void insert_msg(uint8_t id, uint8_t &data[20], uint8_t valid_length);
    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
