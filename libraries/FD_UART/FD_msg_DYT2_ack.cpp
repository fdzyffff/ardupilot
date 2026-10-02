#include "FD_msg_DYT2_ack.h"

FD_msg_DYT2_ack::FD_msg_DYT2_ack(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    reset();
}

void FD_msg_DYT2_ack::reset()
{
    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    _msg.read = 0;
    _msg.length = 0;
    _msg.sum = 0;
}

void FD_msg_DYT2_ack::restart_or_reset(uint8_t temp)
{
    if (temp == PREAMBLE1) {
        _msg.data[0] = temp;
        _msg.read = 1;
        _msg.sum = temp;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
    } else {
        reset();
    }
}

void FD_msg_DYT2_ack::parse(uint8_t temp)
{
    switch (_msg.msg_state) {
    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        if (temp == PREAMBLE1) {
            _msg.data[0] = temp;
            _msg.read = 1;
            _msg.sum = temp;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
        }
        break;
    case FD1UART_msg_parser::FD1UART_PREAMBLE2:
        // 帧头 31 31: 第二字节被 31 34 周期状态帧(31 34 ...)打断时重同步
        if (temp == PREAMBLE2) {
            _msg.data[1] = temp;
            _msg.read = 2;
            _msg.sum = (uint8_t)(_msg.sum + temp);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_ID;
        } else {
            reset();
        }
        break;
    case FD1UART_msg_parser::FD1UART_ID:
        if (temp == MSG_ID) {
            _msg.data[2] = temp;
            _msg.read = 3;
            _msg.sum = (uint8_t)(_msg.sum + temp);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_LEN;
        } else {
            restart_or_reset(temp);
        }
        break;
    case FD1UART_msg_parser::FD1UART_LEN:
        // 只认自检回令(0x06)与设置回令(0x0A)
        if ((temp == MSG_LEN_SELFTEST) || (temp == MSG_LEN_SET)) {
            _msg.data[3] = temp;
            _msg.read = 4;
            _msg.sum = (uint8_t)(_msg.sum + temp);
            _msg.length = temp;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        } else {
            restart_or_reset(temp);
        }
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        _msg.data[_msg.read] = temp;
        _msg.sum = (uint8_t)(_msg.sum + temp);
        _msg.read++;
        if (_msg.read >= (uint16_t)(_msg.length - 1)) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
        }
        break;
    case FD1UART_msg_parser::FD1UART_SUM:
        _msg.data[_msg.read] = temp;
        if (_msg.sum == temp) {
            process_message();
        }
        reset();
        break;
    }
}

void FD_msg_DYT2_ack::process_message(void)
{
    for (uint16_t i = 0; i < _msg.length; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.length = _msg.length;
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_msg_DYT2_ack::swap_message(void)
{
    // 协议为小端字节序，本机同为小端，无需换序。
}

bool FD_msg_DYT2_ack::is_selftest() const
{
    return _msg_1.length == MSG_LEN_SELFTEST;
}

bool FD_msg_DYT2_ack::is_set_ack() const
{
    return _msg_1.length == MSG_LEN_SET;
}

uint8_t FD_msg_DYT2_ack::selftest_result() const
{
    if (!is_selftest()) {
        return 0xFF;
    }
    // 自检回令: 第 5 字节为结果码
    return _msg_1.content.data[4];
}

uint8_t FD_msg_DYT2_ack::set_cmd() const
{
    if (!is_set_ack()) {
        return 0;
    }
    return _msg_1.content.msg.b;
}

int16_t FD_msg_DYT2_ack::set_c() const
{
    if (!is_set_ack()) {
        return 0;
    }
    return _msg_1.content.msg.c;
}

int16_t FD_msg_DYT2_ack::set_d() const
{
    if (!is_set_ack()) {
        return 0;
    }
    return _msg_1.content.msg.d;
}
