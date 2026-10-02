#include "FD_msg_FZ_TY_ack.h"

FD_msg_FZ_TY_ack::FD_msg_FZ_TY_ack(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD_msg_FZ_TY_ack::parse(uint8_t temp)
{
    switch (_msg.msg_state) {
    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        _msg.read = 0;
        _msg.data[0] = temp;
        if (temp == HEAD_1) {
            _msg.read = 1;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
        }
        break;
    case FD1UART_msg_parser::FD1UART_PREAMBLE2:
        if (temp == DEST_UAV) {
            _msg.data[1] = temp;
            _msg.read = 2;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        } else if (temp == HEAD_1) {
            _msg.data[0] = temp;
            _msg.read = 1;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            _msg.read = 0;
        }
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        _msg.data[_msg.read] = temp;
        _msg.read++;
        // 数据1/数据2 冗余，两字节校验一致后进入校验和
        if (_msg.read >= 4U) {
            if (_msg.data[2] != _msg.data[3]) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                _msg.read = 0;
                break;
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
        }
        break;
    case FD1UART_msg_parser::FD1UART_SUM:
        _msg.data[4] = temp;
        if (temp != (uint8_t)(_msg.data[1] + _msg.data[2] + _msg.data[3])) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            _msg.read = 0;
            break;
        }
        _msg.read = 5;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_END;
        break;
    case FD1UART_msg_parser::FD1UART_END:
        _msg.data[5] = temp;
        if (temp == END) {
            process_message();
        }
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        _msg.read = 0;
        break;
    }
}

void FD_msg_FZ_TY_ack::process_message(void)
{
    for (uint16_t i = 0; i < FD_MSG_FZ_TY_ACK_LEN; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_msg_FZ_TY_ack::swap_message(void)
{
    // 单字节状态字，无字节序问题。
}

uint8_t FD_msg_FZ_TY_ack::mode() const
{
    return _msg_1.content.msg.status & 0x03U;
}

uint8_t FD_msg_FZ_TY_ack::arm1() const
{
    return (_msg_1.content.msg.status & 0x04U) ? 1 : 0;
}

uint8_t FD_msg_FZ_TY_ack::arm2() const
{
    return (_msg_1.content.msg.status & 0x08U) ? 1 : 0;
}

uint8_t FD_msg_FZ_TY_ack::pwr_ok() const
{
    return (_msg_1.content.msg.status & 0x10U) ? 1 : 0;
}

uint8_t FD_msg_FZ_TY_ack::locked() const
{
    return (_msg_1.content.msg.status & 0x20U) ? 1 : 0;
}
