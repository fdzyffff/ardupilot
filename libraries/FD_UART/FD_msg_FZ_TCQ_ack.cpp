#include "FD_msg_FZ_TCQ_ack.h"

FD_msg_FZ_TCQ_ack::FD_msg_FZ_TCQ_ack(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_msg_FZ_TCQ_ack::parse(uint8_t temp)
{
    switch (_msg.msg_state) {
    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        _msg.read = 0;
        _msg.data[0] = temp;
        if ((temp == SELFTEST_HEAD1) || (temp == ZG_HEAD1)) {
            _msg.read = 1;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
        }
        break;
    case FD1UART_msg_parser::FD1UART_PREAMBLE2:
        // 自检返回 55 BB；装定返回 AA 55
        if (((_msg.data[0] == SELFTEST_HEAD1) && (temp == SELFTEST_HEAD2)) ||
            ((_msg.data[0] == ZG_HEAD1) && (temp == ZG_HEAD2))) {
            _msg.data[1] = temp;
            _msg.read = 2;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        } else if ((temp == SELFTEST_HEAD1) || (temp == ZG_HEAD1)) {
            _msg.data[0] = temp;
            _msg.read = 1;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            _msg.read = 0;
        }
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        _msg.data[2] = temp;
        _msg.read = 3;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
        break;
    case FD1UART_msg_parser::FD1UART_SUM:
        _msg.data[3] = temp;
        if (temp == (uint8_t)(_msg.data[0] + _msg.data[1] + _msg.data[2])) {
            process_message();
        }
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        _msg.read = 0;
        break;
    }
}

void FD_msg_FZ_TCQ_ack::process_message(void)
{
    for (uint16_t i = 0; i < FD_MSG_FZ_TCQ_ACK_LEN; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_msg_FZ_TCQ_ack::swap_message(void)
{
    // 单字节值，无字节序问题。
}

bool FD_msg_FZ_TCQ_ack::is_selftest() const
{
    return _msg_1.content.msg.head_1 == SELFTEST_HEAD1;
}

int8_t FD_msg_FZ_TCQ_ack::selftest_result() const
{
    if (!is_selftest()) {
        return 0;
    }
    if (_msg_1.content.msg.value == SELFTEST_OK) {
        return 1;
    }
    if (_msg_1.content.msg.value == SELFTEST_FAIL) {
        return 2;
    }
    return 0;
}

uint8_t FD_msg_FZ_TCQ_ack::zg_zd() const
{
    if (is_selftest()) {
        return 0;
    }
    return _msg_1.content.msg.value;
}
