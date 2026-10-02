#include "FD_msg_DET_LK.h"

FD_msg_DET_LK::FD_msg_DET_LK(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    reset();
}

void FD_msg_DET_LK::reset()
{
    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    _msg.read = 0;
    _msg.count = 0;
}

void FD_msg_DET_LK::parse(uint8_t temp)
{
    switch (_msg.msg_state) {
    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        _msg.read = 0;
        _msg.count = 0;
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
            _msg.msg_state = FD1UART_msg_parser::FD1UART_COUNT;
        } else if (temp == PREAMBLE1) {
            // 连续帧头: 当前字节作为新的帧头1
            _msg.data[0] = temp;
            _msg.read = 1;
        } else {
            reset();
        }
        break;
    case FD1UART_msg_parser::FD1UART_COUNT:
        // 目标个数越界(含 0，文档中无目标不发帧): 丢帧并重同步
        if ((temp == 0U) || (temp > MAX_TARGETS)) {
            if (temp == PREAMBLE1) {
                _msg.data[0] = temp;
                _msg.read = 1;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            } else {
                reset();
            }
            break;
        }
        _msg.count = temp;
        _msg.data[2] = temp;
        _msg.read = 3;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        _msg.data[_msg.read] = temp;
        _msg.read++;
        if (_msg.read >= (uint16_t)(3U + _msg.count)) {
            process_message();
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            _msg.read = 0;
        }
        break;
    }
}

void FD_msg_DET_LK::process_message(void)
{
    for (uint16_t i = 0; i < FD_MSG_DET_LK_LEN; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_msg_DET_LK::swap_message(void)
{
    // 距离为单字节，无字节序问题。
}

float FD_msg_DET_LK::distance_m(uint8_t idx) const
{
    if (idx >= _msg_1.content.msg.count) {
        return 0.0f;
    }
    return (float)_msg_1.content.msg.dist[idx] * 0.25f;
}
