#include "FD1_msg_BIMCMD.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_BIMCMD::FD1_msg_BIMCMD(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_BIMCMD::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %d",_msg.msg_state, temp);
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.sum_check += temp;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.length = _msg_1.length;
                _msg.read = 2;
                _msg.sum_check += temp;
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
            _msg.read++;
            _msg.sum_check += temp;

            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %x - %x", _msg.msg_state, temp, (_msg.sum_check>>8));
            if (temp == _msg.sum_check)
            {
                process_message();
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "temp: %d, _msg.sum_check: %d",temp, _msg.sum_check);
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            break;
    }
}

void FD1_msg_BIMCMD::process_message(void)
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

void FD1_msg_BIMCMD::swap_message(void)
{
    // _msg_1.content.msg.psi = swap_message_uint32_t(_msg_1.content.msg.psi);
    // _msg_1.content.msg.ps = swap_message_uint32_t(_msg_1.content.msg.ps);
    // _msg_1.content.msg.qci = swap_message_int32_t(_msg_1.content.msg.qci);
    // _msg_1.content.msg.qc = swap_message_int32_t(_msg_1.content.msg.qc);
    // _msg_1.content.msg.hp = swap_message_int32_t(_msg_1.content.msg.hp);
    // _msg_1.content.msg.hpr = swap_message_int16_t(_msg_1.content.msg.hpr);
    // _msg_1.content.msg.ts = swap_message_int16_t(_msg_1.content.msg.ts);
    // _msg_1.content.msg.tt = swap_message_int16_t(_msg_1.content.msg.tt);
    // _msg_1.content.msg.mi = swap_message_uint16_t(_msg_1.content.msg.mi);
    // _msg_1.content.msg.vi = swap_message_uint16_t(_msg_1.content.msg.vi);
    // _msg_1.content.msg.vt = swap_message_uint16_t(_msg_1.content.msg.vt);
    // _msg_1.content.msg.adr = swap_message_uint16_t(_msg_1.content.msg.adr);
    // _msg_1.content.msg.aoai1 = swap_message_int16_t(_msg_1.content.msg.aoai1);
    // _msg_1.content.msg.aoai2 = swap_message_int16_t(_msg_1.content.msg.aoai2);
    // _msg_1.content.msg.aoat1 = swap_message_int16_t(_msg_1.content.msg.aoat1);
    // _msg_1.content.msg.aoat2 = swap_message_int16_t(_msg_1.content.msg.aoat2);
    // _msg_1.content.msg.aosi1 = swap_message_int16_t(_msg_1.content.msg.aosi1);
    // _msg_1.content.msg.aosi2 = swap_message_int16_t(_msg_1.content.msg.aosi2);
    // _msg_1.content.msg.aost1 = swap_message_int16_t(_msg_1.content.msg.aost1);
    // _msg_1.content.msg.aost2 = swap_message_int16_t(_msg_1.content.msg.aost2);
    // _msg_1.content.msg.faultword = swap_message_uint16_t(_msg_1.content.msg.faultword);
    // _msg_1.content.msg.datavalid = swap_message_uint32_t(_msg_1.content.msg.datavalid);
    // _msg_1.content.msg.coffpress_k0 = swap_message_uint16_t(_msg_1.content.msg.coffpress_k0);
    // _msg_1.content.msg.coffpress_k1 = swap_message_uint16_t(_msg_1.content.msg.coffpress_k1);
    // _msg_1.content.msg.coffpress_k2 = swap_message_uint16_t(_msg_1.content.msg.coffpress_k2);
    // _msg_1.content.msg.coffpress_k3 = swap_message_uint16_t(_msg_1.content.msg.coffpress_k3);
    // _msg_1.content.msg.coffangle_k0 = swap_message_int16_t(_msg_1.content.msg.coffangle_k0);
    // _msg_1.content.msg.coffangle_k1 = swap_message_int16_t(_msg_1.content.msg.coffangle_k1);
    // _msg_1.content.msg.coffangle_k2 = swap_message_int16_t(_msg_1.content.msg.coffangle_k2);
    // _msg_1.content.msg.coffangle_k3 = swap_message_int16_t(_msg_1.content.msg.coffangle_k3);
}

void FD1_msg_BIMCMD::sum_check()
{
    int16_t i = 0;
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.sum = 0;

    for (i = 0; i < _msg_1.length-1; i ++) {
        _msg_1.content.msg.sum += _msg_1.content.data[i];
    }
} 
