#include "HB1_power2apm.h"

HB1_power2apm::HB1_power2apm(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_power2apm::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case HB1UART_msg_parser::HB1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            if (temp == PREAMBLE1) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE2;
                _msg.data[_msg.read] = temp;
                _msg.read++;
            }
            break;
        case HB1UART_msg_parser::HB1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.header.head_1 = PREAMBLE1;
                _msg.header.head_2 = PREAMBLE2;
                _msg.length = _msg_1.length;
                _msg.data[_msg.read] = temp;
                _msg.read++;
                _msg.msg_state = HB1UART_msg_parser::HB1UART_DATA;
            }
            else
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
            }
            break;
        case HB1UART_msg_parser::HB1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;

            _msg.read++;
            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_SUM;
            }
            break;
        case HB1UART_msg_parser::HB1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
            _msg.sum_check = crc8_itu(_msg.data, _msg.read);

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void HB1_power2apm::process_message(void)
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

void HB1_power2apm::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[4], _msg_1.content.data[5]);
    // swap_message_sub(_msg_1.content.data[6], _msg_1.content.data[7]);
    // swap_message_sub(_msg_1.content.data[9], _msg_1.content.data[10]);
    // swap_message_sub(_msg_1.content.data[13], _msg_1.content.data[14]);
    // swap_message_sub(_msg_1.content.data[16], _msg_1.content.data[17]);
    // swap_message_sub(_msg_1.content.data[18], _msg_1.content.data[19]);
    // swap_message_sub(_msg_1.content.data[20], _msg_1.content.data[21]);
    // swap_message_sub(_msg_1.content.data[22], _msg_1.content.data[23]);
    ;
}

uint8_t HB1_power2apm::crc8_itu(uint8_t *data, uint8_t len)
{
    uint8_t crca = data[0];
    uint8_t crcb = 0;
    for(int i = 0; i < len; i++)
    {
        if(i == 0)
        {
            crca = crc8_itu_table[(data[i] ^ 0x00)];
            crcb = crca;
        }
        else
        {
            crca = crc8_itu_table[(data[i] ^ crcb)];
            crcb = crca;
        }
    }
    return (crcb ^ 0x55);
}
