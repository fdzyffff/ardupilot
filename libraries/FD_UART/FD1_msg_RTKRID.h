#pragma once

#include "FD1_message.h"

// 无人机远程识别模块（RID）ASCII 命令协议
//   模块：UM982 + 4G 通讯一体化板卡（说明书 V3.0）
//   写命令：*NAME_W:value#    读命令：*NAME_R:#    应答：*NAME_R:value#
//   \r\n 结尾，无帧头、无校验、无长度字段
//
// 与老的 FD1_msg_RID（203 字节二进制国标帧）完全无关，两者靠 SERIALn_PROTOCOL 区分：
//   SERIALn_PROTOCOL = 50 → RID    ：老模块，二进制帧，FD1_msg_RID（本类不改它）
//   SERIALn_PROTOCOL = 51 → RTKRID ：本类，ASCII 命令
//
// 数据分工：位置/速度/航迹角/时间戳由模块自取 UM982 COM3 的 GGA/RMC，
//           飞控只负责身份（写一次）+ 运行状态（1 Hz）。
//
// 时序：每 15 s 一轮探测 —— 先读回，读回与期望一致即认定已配置，本次上电不再探测、不再写；
//       不一致则本轮把静态项写一遍，下一轮再读回确认。
//       模块侧“上电 5 s 内可写”的窗口由厂商取消，本类不含任何窗口相关常量。

#define FD1_MSG_RTKRID_PROBE_MS     15000U   // 探测/写入周期
#define FD1_MSG_RTKRID_STATUS_MS     1000U   // UA_STATUS 发送周期
#define FD1_MSG_RTKRID_RX_MAX          48U   // 应答行缓冲上限
#define FD1_MSG_RTKRID_CMD_MAX         48U   // 单条命令缓冲上限

#define FD1_MSG_RTKRID_UA_TYPE          2    // 飞机类型
#define FD1_MSG_RTKRID_UA_CLASS         3    // UA 等级
#define FD1_MSG_RTKRID_UA_CATEGORY      3    // UA 运行类别
#define FD1_MSG_RTKRID_ATTRIBUTION      2    // 等级分别归属区域

class FD1_msg_RTKRID : public FD1_message {
public:
    FD1_msg_RTKRID();

    /* Do not allow copies */
    FD1_msg_RTKRID(const FD1_msg_RTKRID &other) = delete;
    FD1_msg_RTKRID &operator=(const FD1_msg_RTKRID&) = delete;

    void process_message(void) override { ; }   // 由 get_pending() 拉取驱动
    void parse(uint8_t temp) override;          // 行缓冲，解析 *NAME:value#\r\n
    void swap_message() override { ; }          // ASCII 命令无字节序问题

    // 复位状态机；now_ms 传 millis()，上电后第一拍即探测
    void init(uint32_t now_ms);

    // 身份来源（UAS 唯一产品识别码 ≤20 字符、实名登记标志 → 取 uas_number[12..19]）
    void set_identity(const char *serial_number, const char *uas_number);

    // 拉取一条待发命令，返回字节数；返回 0 = 本轮无命令
    uint16_t get_pending(char *buf, uint16_t size, uint8_t ua_status);

    bool configured() const { return _configured; }    // true = 本次上电已写完并读回确认
    bool module_seen() const { return _module_seen; }  // true = 收到过合法应答

private:
    enum class Stage : uint8_t {
        IDLE = 0,       // 等下一个 15 s 探测点
        PROBE,          // 本轮探测/写入进行中
    };

    // 一轮 burst 的命令序号：先读回（0,1），不一致再写静态项（2..7）
    enum Seq : uint8_t {
        SEQ_READ_ID = 0,
        SEQ_READ_TYPE,
        SEQ_WRITE_ID,
        SEQ_WRITE_RNM,
        SEQ_WRITE_TYPE,
        SEQ_WRITE_CLASS,
        SEQ_WRITE_CATEGORY,
        SEQ_WRITE_ATTRIBUTION,
        SEQ_NUM
    };

    void handle_line(const char *line, uint8_t len);
    static uint16_t make_cmd(char *buf, uint16_t size, const char *fmt, ...);

    Stage    _stage;
    uint8_t  _seq;              // 本轮 burst 的命令序号
    bool     _configured;       // 已配置完成（本次上电不再探测）
    bool     _module_seen;      // 收到过模块应答
    bool     _id_matched;       // 读回的 UAS_ID 与期望一致
    uint32_t _last_status_ms;
    uint32_t _last_probe_ms;
    char     _serial_number[21];    // UAS 唯一产品识别码
    char     _uas_number[21];       // 实名登记标志源（取 [12..19]）
    char     _rx[FD1_MSG_RTKRID_RX_MAX];
    uint8_t  _rx_len;
};
