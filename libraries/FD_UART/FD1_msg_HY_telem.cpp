#include "FD1_msg_HY_telem.h"

// ---------------------------------------------------------------------------
// 00 81 测偏数据(脱靶量)
// ---------------------------------------------------------------------------

FD1_msg_HY_miss::FD1_msg_HY_miss(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_miss::parse(uint8_t temp)
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
            _msg.sum = temp;                 // 校验从 CMD0 起累加
            _msg.read = 3;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_CMD1;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_CMD1:
        if (temp == MSG_CMD1) {
            _msg.data[3] = temp;
            _msg.sum += temp;
            _msg.read = 4;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_LEN;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_LEN:
        if (temp != MSG_LEN) {               // 定长帧，长度不符即丢弃
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[4] = temp;
        _msg.sum += temp;
        _msg.length = (uint16_t)temp + 7;
        _msg.read = 5;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        if (_msg.read > sizeof(_msg.data) - 2) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[_msg.read] = temp;
        _msg.sum += temp;
        _msg.read++;
        if (_msg.read >= _msg.length - 2) {
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

void FD1_msg_HY_miss::process_message(void)
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

void FD1_msg_HY_miss::swap_message(void)
{
    ;   // 协议为小端，与目标平台一致，无需换序（同 DYT 处理）
}

// ---------------------------------------------------------------------------
// 00 82 AI目标检测（变长）
// ---------------------------------------------------------------------------

FD1_msg_HY_ai::FD1_msg_HY_ai(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_ai::parse(uint8_t temp)
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
        if (temp == MSG_CMD1) {
            _msg.data[3] = temp;
            _msg.sum += temp;
            _msg.read = 4;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_LEN;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_LEN:
        if ((temp < 3) || (temp > FD1_MSG_HY_AI_MAX_DATA)) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[4] = temp;
        _msg.sum += temp;
        _msg.length = (uint16_t)temp + 7;
        _msg.read = 5;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        if (_msg.read > sizeof(_msg.data) - 2) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[_msg.read] = temp;
        _msg.sum += temp;
        _msg.read++;
        if (_msg.read >= _msg.length - 2) {
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

void FD1_msg_HY_ai::process_message(void)
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

void FD1_msg_HY_ai::swap_message(void)
{
    ;
}

// ---------------------------------------------------------------------------
// 00 83 心跳
// ---------------------------------------------------------------------------

FD1_msg_HY_hb::FD1_msg_HY_hb(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_HY_hb::parse(uint8_t temp)
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
        if (temp == MSG_CMD1) {
            _msg.data[3] = temp;
            _msg.sum += temp;
            _msg.read = 4;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_LEN;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    case FD1UART_msg_parser::FD1UART_LEN:
        if (temp != MSG_LEN) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[4] = temp;
        _msg.sum += temp;
        _msg.length = (uint16_t)temp + 7;
        _msg.read = 5;
        _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        break;
    case FD1UART_msg_parser::FD1UART_DATA:
        if (_msg.read > sizeof(_msg.data) - 2) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
        }
        _msg.data[_msg.read] = temp;
        _msg.sum += temp;
        _msg.read++;
        if (_msg.read >= _msg.length - 2) {
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

void FD1_msg_HY_hb::process_message(void)
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

void FD1_msg_HY_hb::swap_message(void)
{
    ;
}
