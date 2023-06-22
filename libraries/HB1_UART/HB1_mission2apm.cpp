#include "HB1_mission2apm.h"
#include <GCS_MAVLink/GCS.h>

HB1_mission2apm::HB1_mission2apm(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void HB1_mission2apm::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "Mission2APM:%x",temp);
    switch (_msg.msg_state)
    {
        default:
        case HB1UART_msg_parser::HB1UART_PREAMBLE1:
            // gcs().send_text(MAV_SEVERITY_INFO, "HB1UART_PREAMBLE1");
            _msg.read = 0;
            _msg.sum_check = 0;
            if (temp == PREAMBLE1) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE2;
                _msg.data[_msg.read] = temp;
                _msg.read++;
            }
            break;
        case HB1UART_msg_parser::HB1UART_PREAMBLE2:
            // gcs().send_text(MAV_SEVERITY_INFO, "HB1UART_PREAMBLE2");
            if (temp == PREAMBLE2)
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_INDEX;
                _msg.data[_msg.read] = temp;
                _msg.read++;
            }
            else
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
            }
            break;
        case HB1UART_msg_parser::HB1UART_INDEX:
            // gcs().send_text(MAV_SEVERITY_INFO, "HB1UART_INDEX");
            _msg.header.head_1 = PREAMBLE1;
            _msg.header.head_2 = PREAMBLE2;
            _msg.sum_check = 0;
            _msg.sum_check += temp;
            _msg.header.index = temp;
            switch (_msg.header.index) {
                case INDEX1:
                    _msg.length = _msg_1.length;
                    _msg.data[_msg.read] = temp;
                    _msg.read++;
                    _msg.msg_state = HB1UART_msg_parser::HB1UART_DATA;
                    break;
                default:
                    _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
                    break;
            }
            break;

        case HB1UART_msg_parser::HB1UART_DATA:
            // gcs().send_text(MAV_SEVERITY_INFO, "HB1UART_DATA");
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;

            _msg.sum_check += temp;

            _msg.read++;
            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_SUM;
            }
            break;
        case HB1UART_msg_parser::HB1UART_SUM:
            // gcs().send_text(MAV_SEVERITY_INFO, "HB1UART_SUM");
            _msg.data[_msg.read] = temp;
            _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void HB1_mission2apm::process_message(void)
{
    int16_t i = 0;
    switch (_msg.header.index) {
        case INDEX1:
            for (i = 0; i < _msg_1.length; i ++) {
                _msg_1.content.data[i] = _msg.data[i];
            }
            swap_message();
            _msg_1.print = true;
            _msg_1.updated = true;
            _msg_1.need_send = false;
            break;
        default:
            break;
    }

}
