#include "FD_msg_DYT2_telem.h"

FD_msg_DYT2_telem::FD_msg_DYT2_telem(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    reset();
}

void FD_msg_DYT2_telem::reset()
{
    _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    _msg.read = 0;
    _msg.length = MSG_LEN;
    _msg.sum = 0;
}

// 校验/标识不符时的重同步: 当前字节若为帧头1 就作为新帧起点，否则整帧丢弃
void FD_msg_DYT2_telem::restart_or_reset(uint8_t temp)
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

void FD_msg_DYT2_telem::parse(uint8_t temp)
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
        if (temp == PREAMBLE2) {
            _msg.data[1] = temp;
            _msg.read = 2;
            _msg.sum = (uint8_t)(_msg.sum + temp);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_ID;
        } else if (temp == PREAMBLE1) {
            // 31 31 回令帧（本类不处理, 由 FD_msg_DYT2_ack 解析）: 当前字节作为新帧头1
            _msg.data[0] = temp;
            _msg.read = 1;
            _msg.sum = temp;
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
        if (temp == MSG_LEN) {
            _msg.data[3] = temp;
            _msg.read = 4;
            _msg.sum = (uint8_t)(_msg.sum + temp);
            _msg.length = MSG_LEN;
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

void FD_msg_DYT2_telem::process_message(void)
{
    for (uint16_t i = 0; i < FD_MSG_DYT2_TELEM_LEN; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.length = FD_MSG_DYT2_TELEM_LEN;
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD_msg_DYT2_telem::swap_message(void)
{
    // 协议为小端字节序，本机同为小端，无需换序。
}

float FD_msg_DYT2_telem::frame_pitch_deg() const
{
    return (float)_msg_1.content.msg.frame_pitch * 0.1f;
}

float FD_msg_DYT2_telem::frame_yaw_deg() const
{
    return (float)_msg_1.content.msg.frame_yaw * 0.1f;
}

float FD_msg_DYT2_telem::offset_pitch_deg() const
{
    return (float)_msg_1.content.msg.offset_pitch * 0.01f;
}

float FD_msg_DYT2_telem::offset_yaw_deg() const
{
    return (float)_msg_1.content.msg.offset_yaw * 0.01f;
}

float FD_msg_DYT2_telem::los_rate_pitch_dps() const
{
    return (float)_msg_1.content.msg.los_rate_pitch * 0.01f;
}

float FD_msg_DYT2_telem::los_rate_yaw_dps() const
{
    return (float)_msg_1.content.msg.los_rate_yaw * 0.01f;
}

float FD_msg_DYT2_telem::ins_rate_x_dps() const
{
    return (float)_msg_1.content.msg.ins_rate_x * 0.01f;
}

float FD_msg_DYT2_telem::ins_rate_y_dps() const
{
    return (float)_msg_1.content.msg.ins_rate_y * 0.01f;
}

float FD_msg_DYT2_telem::ins_rate_z_dps() const
{
    return (float)_msg_1.content.msg.ins_rate_z * 0.01f;
}

bool FD_msg_DYT2_telem::tracking() const
{
    return (_msg_1.content.msg.track & 0x01U) != 0U;
}

bool FD_msg_DYT2_telem::detected() const
{
    return (_msg_1.content.msg.detect & 0x01U) != 0U;
}

uint8_t FD_msg_DYT2_telem::dyt_state() const
{
    return _msg_1.content.msg.state;
}

uint8_t FD_msg_DYT2_telem::zoom() const
{
    return _msg_1.content.msg.zoom;
}

uint8_t FD_msg_DYT2_telem::record() const
{
    return _msg_1.content.msg.record;
}

uint8_t FD_msg_DYT2_telem::storage_percent() const
{
    return _msg_1.content.msg.storage;
}

uint8_t FD_msg_DYT2_telem::power_state() const
{
    return _msg_1.content.msg.power;
}

uint8_t FD_msg_DYT2_telem::frame_count() const
{
    return _msg_1.content.msg.frame_count;
}
