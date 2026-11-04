#include "FD1_msg_RTKRID.h"

#include <AP_HAL/AP_HAL.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

FD1_msg_RTKRID::FD1_msg_RTKRID(void)
{
    _enable = false;
    _stage = Stage::IDLE;
    _seq = 0;
    _configured = false;
    _module_seen = false;
    _id_matched = false;
    _last_status_ms = 0;
    _last_probe_ms = 0;
    memset(_serial_number, 0, sizeof(_serial_number));
    memset(_uas_number, 0, sizeof(_uas_number));
    memset(_rx, 0, sizeof(_rx));
    _rx_len = 0;
}

void FD1_msg_RTKRID::init(uint32_t now_ms)
{
    _stage = Stage::IDLE;
    _seq = 0;
    _configured = false;
    _module_seen = false;
    _id_matched = false;
    _last_status_ms = now_ms;
    // 无符号回绕运算：now_ms - _last_probe_ms 立即 >= PROBE_MS，上电第一拍就探测
    _last_probe_ms = now_ms - FD1_MSG_RTKRID_PROBE_MS;
    memset(_rx, 0, sizeof(_rx));
    _rx_len = 0;
    set_enable();
}

void FD1_msg_RTKRID::set_identity(const char *serial_number, const char *uas_number)
{
    memset(_serial_number, 0, sizeof(_serial_number));
    memset(_uas_number, 0, sizeof(_uas_number));
    if (serial_number != nullptr) {
        strncpy(_serial_number, serial_number, sizeof(_serial_number) - 1);
    }
    if (uas_number != nullptr) {
        strncpy(_uas_number, uas_number, sizeof(_uas_number) - 1);
    }
}

uint16_t FD1_msg_RTKRID::make_cmd(char *buf, uint16_t size, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, size, fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return 0;
    }
    if (n >= (int)size) {
        return (uint16_t)(size - 1);
    }
    return (uint16_t)n;
}

void FD1_msg_RTKRID::parse(uint8_t temp)
{
    if (!_enable) {
        return;
    }

    const char c = (char)temp;

    if (c == '\r') {
        return;
    }

    if (c == '\n') {
        _rx[_rx_len] = 0;
        handle_line(_rx, _rx_len);
        _rx_len = 0;
        return;
    }

    if (_rx_len >= (FD1_MSG_RTKRID_RX_MAX - 1)) {
        // 超长行（模块日志/异常数据）整行丢弃，避免拼进后续应答
        _rx_len = 0;
        return;
    }

    _rx[_rx_len++] = c;
}

// 模块上电会输出 +EIND:128、$STATUS,5001、GGA/RMC 等日志，只认 '*' 开头的命令应答
void FD1_msg_RTKRID::handle_line(const char *line, uint8_t len)
{
    if (len < 4 || line[0] != '*') {
        return;
    }

    if (strncmp(line, "*UAS_ID_R:", 10) == 0) {
        _module_seen = true;

        const char *p = &line[10];
        char value[21];
        uint8_t i = 0;
        while ((p[i] != '#') && (p[i] != 0) && (i < 20)) {
            value[i] = p[i];
            i++;
        }
        value[i] = 0;

        const size_t expect_len = strnlen(_serial_number, sizeof(_serial_number));
        _id_matched = (expect_len > 0) && ((size_t)i == expect_len) &&
                      (strncmp(value, _serial_number, i) == 0);
        return;
    }

    if (strncmp(line, "*UA_TYPE_R:", 11) == 0) {
        _module_seen = true;
        return;
    }
}

uint16_t FD1_msg_RTKRID::get_pending(char *buf, uint16_t size, uint8_t ua_status)
{
    if (!_enable || (buf == nullptr) || (size < 8)) {
        return 0;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // 运行状态：1 Hz，任何时刻可写，与是否探测成功无关
    if ((now_ms - _last_status_ms) >= FD1_MSG_RTKRID_STATUS_MS) {
        _last_status_ms = now_ms;
        return make_cmd(buf, size, "*UA_STATUS_W:%u#\r\n", (unsigned)ua_status);
    }

    if (_configured) {
        return 0;                   // 写完且读回一致：本次上电不再探测
    }

    if (_stage == Stage::IDLE) {
        if ((now_ms - _last_probe_ms) < FD1_MSG_RTKRID_PROBE_MS) {
            return 0;
        }
        _stage = Stage::PROBE;
        _seq = SEQ_READ_ID;
        _id_matched = false;
    }

    // 本轮读回已与期望一致 → 认定配置完成，剩余写命令不再发
    if (_id_matched) {
        _configured = true;
        _stage = Stage::IDLE;
        _seq = 0;
        _last_probe_ms = now_ms;
        return 0;
    }

    uint16_t n = 0;

    switch (_seq) {
    case SEQ_READ_ID:
        n = make_cmd(buf, size, "*UAS_ID_R:#\r\n");
        break;
    case SEQ_READ_TYPE:
        n = make_cmd(buf, size, "*UA_TYPE_R:#\r\n");
        break;
    case SEQ_WRITE_ID:
        if (_serial_number[0] != 0) {
            n = make_cmd(buf, size, "*UAS_ID_W:%.20s#\r\n", _serial_number);
        }
        break;
    case SEQ_WRITE_RNM:
        if (strnlen(_uas_number, sizeof(_uas_number)) >= 20) {
            n = make_cmd(buf, size, "*UASID_RNM_W:%.8s#\r\n", &_uas_number[12]);
        }
        break;
    case SEQ_WRITE_TYPE:
        n = make_cmd(buf, size, "*UA_TYPE_W:%u#\r\n", (unsigned)FD1_MSG_RTKRID_UA_TYPE);
        break;
    case SEQ_WRITE_CLASS:
        n = make_cmd(buf, size, "*UA_CLASS_W:%u#\r\n", (unsigned)FD1_MSG_RTKRID_UA_CLASS);
        break;
    case SEQ_WRITE_CATEGORY:
        n = make_cmd(buf, size, "*UA_CATEGORY_W:%u#\r\n", (unsigned)FD1_MSG_RTKRID_UA_CATEGORY);
        break;
    default:
        n = make_cmd(buf, size, "*ATTRIBUTION_W:%u#\r\n", (unsigned)FD1_MSG_RTKRID_ATTRIBUTION);
        break;
    }

    if (++_seq >= SEQ_NUM) {
        _seq = 0;
        _stage = Stage::IDLE;
        _last_probe_ms = now_ms;
    }

    return n;
}
