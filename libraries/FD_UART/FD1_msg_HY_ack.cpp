#include "FD1_msg_HY_ack.h"

FD1_msg_HY_ack::FD1_msg_HY_ack(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_ack::parse(uint8_t temp)
{
    switch (_msg.msg_state) {
    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        _msg.read = 0;
        _msg.sum = 0;
        _msg.data[0] = temp;
        if (temp == PREAMBLE1) {
            _msg.read = 1;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
        }
        break;
    case FD1UART_msg_parser::FD1UART_PREAMBLE2:
        if (temp == PREAMBLE2) {
            _msg.data[1] = temp;
            _msg.read = 2;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_CMD0;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_CMD0:
        if (temp == MSG_CMD0) {
            _msg.data[2] = temp;
            _msg.sum = temp;
            _msg.read = 3;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_CMD1;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_CMD1:
        if ((temp == MSG_CMD1_DETECT) || (temp == MSG_CMD1_AUTOLOCK)) {
            _msg.data[3] = temp;
            _msg.sum += temp;
            _msg.read = 4;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_LEN;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_LEN:
        _msg.data[4] = temp;
        _msg.sum += temp;
        _msg.length = (uint16_t)temp + 7U;
        if (_msg.length > FD1_MSG_HY_ACK_MAX_LEN) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.read = 5;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        if (_msg.read > sizeof(_msg.data) - 2U) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[_msg.read] = temp;
        _msg.sum += temp;
        _msg.read++;
        if (_msg.read >= _msg.length - 2U) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
        }
        break;
    case FD1UART_msg_parser::FD1UART_SUM:
        _msg.data[_msg.read] = temp;
        _msg.read++;
        _msg.msg_state = (_msg.sum == temp) ? FD1UART_msg_parser::FD1UART_END
                                            : FD1UART_msg_parser::FD1UART_PREAMBLE1;
        break;
    case FD1UART_msg_parser::FD1UART_END:
        _msg.data[_msg.read] = temp;
        if (temp == MSG_END) {
            process_message();
        }
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        break;
    }
}

void FD1_msg_HY_ack::process_message(void)
{
    _msg_1.length = _msg.length;
    for (uint16_t i = 0; i < _msg_1.length; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_HY_ack::swap_message(void)
{
    // 协议与飞控均为小端字节序。
}
