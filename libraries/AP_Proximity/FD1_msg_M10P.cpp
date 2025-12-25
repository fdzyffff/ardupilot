#include "FD1_msg_M10P.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_M10P::FD1_msg_M10P(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_M10P::parse(uint8_t temp)
{
    // if (_msg.msg_state > 0) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "msg_state: %d, temp %x",_msg.msg_state, temp);
    // }
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
                _msg.length = 10;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.read = 2;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-2) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }

            _msg.data[_msg.read] = temp;

            if (_msg.read == 3) {
                _msg.length = ((uint16_t)(_msg.data[_msg.read-1] << 8) + (uint16_t)_msg.data[_msg.read] );
                // gcs().send_text(MAV_SEVERITY_INFO, "_msg.length: %d",_msg.length);
            }

            _msg.read++;

            if (_msg.read >= (_msg.length - 2))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_POSTAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_POSTAMBLE1:
            // gcs().send_text(MAV_SEVERITY_INFO, "1 State: %d, Byte: %x - %x",_msg.msg_state, temp, POSTAMBLE1);
            if (temp == POSTAMBLE1) {
                _msg.data[_msg.read] = temp;
                _msg.read++;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_POSTAMBLE2;
            } else {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_POSTAMBLE2:
            // gcs().send_text(MAV_SEVERITY_INFO, "2 State: %d, Byte: %x - %x",_msg.msg_state, temp, POSTAMBLE2);
            if (temp == POSTAMBLE2)
            {
                _msg.data[_msg.read] = temp;
                process_message();
            }
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_M10P::process_message(void)
{
    int16_t i = 0;

    for (i = 0; i < _msg_1.length; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_M10P::swap_message(void)
{
    _msg_1.content.msg.length = swap_message_uint16_t(_msg_1.content.msg.length);
    _msg_1.content.msg.angle = swap_message_uint16_t(_msg_1.content.msg.angle);
    _msg_1.content.msg.speed = swap_message_uint16_t(_msg_1.content.msg.speed);
    // gcs().send_text(MAV_SEVERITY_INFO, "length: %d, angle: %d, speed: %d",_msg_1.content.msg.length, _msg_1.content.msg.angle, _msg_1.content.msg.speed);
}
