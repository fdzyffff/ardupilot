#include "HB1_power2apm.h"
#include <GCS_MAVLink/GCS.h>

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
            if (temp == PREAMBLE1) {
                _msg.msg_state = HB1UART_msg_parser::HB1UART_DATA;
                _msg.data[_msg.read] = temp;
                _msg.read++;
            }
            break;
        case HB1UART_msg_parser::HB1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
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
        case HB1UART_msg_parser::HB1UART_SUM: {
            _msg.data[_msg.read] = temp;
            _msg.read++;
            uint8_t i = 0;
            uint8_t k = 0;
            uint8_t crc8 = 0;
            for (i = 0; i < _msg.read-1; i++) {
                k = _msg.data[i]^crc8;
                crc8 = 0;
                if (k & 0x01) {crc8 ^= 0x5E;}
                if (k & 0x02) {crc8 ^= 0xBC;}
                if (k & 0x04) {crc8 ^= 0x61;}
                if (k & 0x08) {crc8 ^= 0xC2;}
                if (k & 0x10) {crc8 ^= 0x9D;}
                if (k & 0x20) {crc8 ^= 0x23;}
                if (k & 0x40) {crc8 ^= 0x46;}
                if (k & 0x80) {crc8 ^= 0x8C;}
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "KKK %d", i);
            // gcs().send_text(MAV_SEVERITY_INFO, "FFF %x, %x", temp, crc8);
            if (temp == k) {
                process_message();
            }
            _msg.msg_state = HB1UART_msg_parser::HB1UART_PREAMBLE1;
            break;
        }
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

// void HB1_power2apm::make_sum()
// {
//     uint8_t i = 0;
//     uint8_t k = 0;
//     uint8_t crc8 = 0;
//     for (i = 0; i < _msg_1.length-1; i++) {
//         k = _msg_1.content.data[i]^crc8;
//         crc8 = 0;
//         if (k & 0x01) {crc8 ^= 0x5E;}
//         if (k & 0x02) {crc8 ^= 0xBC;}
//         if (k & 0x04) {crc8 ^= 0x61;}
//         if (k & 0x08) {crc8 ^= 0xC2;}
//         if (k & 0x10) {crc8 ^= 0x9D;}
//         if (k & 0x20) {crc8 ^= 0x23;}
//         if (k & 0x40) {crc8 ^= 0x46;}
//         if (k & 0x80) {crc8 ^= 0x8C;}
//     }
//     _msg_1.content.msg.crc = k;
// }
