#include "FD1_msg_engine_response.h"
// #include <GCS_MAVLink/GCS.h>

FD1_msg_engine_response::FD1_msg_engine_response(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_engine_response::parse(uint8_t temp)
{
    if (AP_HAL::millis() - _last_byte_ms > 100) {
        _last_byte_ms = AP_HAL::millis();
        _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
    }

    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum = 0;
            _msg.data[_msg.read] = temp;// 0
            if (temp == 0x00) {
                _msg.read++;
                _msg.sum += temp;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == 0x52)
            {
                _msg.data[_msg.read] = temp;// 1
                _msg.read++;
                _msg.sum = 0;
                _msg.length = FD1_MSG_ENGINE_RESPONSE_LEN;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-4) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;
            _msg.sum += temp;
            _msg.read++;

            if (_msg.read >= (_msg.length - 4))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
                _msg_crc_count = 0;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            if (_msg_crc_count < 4) {
                _msg.data[_msg.read] = temp;
                _msg.read++;
                _msg_crc_count++;
            } else {
                // if (crc_ok) {
                    process_message();
                // }
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "sum: %d, sum_in: %d",_msg.sum, temp);

            break;
    }
}

void FD1_msg_engine_response::process_message(void)
{
    int16_t i = 0;

    _msg_1.length = _msg.length;
    for (i = 0; i <= _msg_1.length-1; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_engine_response::make_sum()
{
    ;
}

void FD1_msg_engine_response::swap_message(void)
{
    _msg_1.content.msg.size = swap_message_uint16_t(_msg_1.content.msg.size);
    _msg_1.content.msg.seconds = swap_message_uint16_t(_msg_1.content.msg.seconds);
    _msg_1.content.msg.pulsewidth1 = swap_message_uint16_t(_msg_1.content.msg.pulsewidth1);
    _msg_1.content.msg.pulsewidth2 = swap_message_uint16_t(_msg_1.content.msg.pulsewidth2);
    _msg_1.content.msg.rpm = swap_message_uint16_t(_msg_1.content.msg.rpm);
    _msg_1.content.msg.advance = swap_message_uint16_t(_msg_1.content.msg.advance);
    _msg_1.content.msg.barometer = swap_message_uint16_t(_msg_1.content.msg.barometer);
    _msg_1.content.msg.map = swap_message_uint16_t(_msg_1.content.msg.map);
    _msg_1.content.msg.mat = swap_message_uint16_t(_msg_1.content.msg.mat);
    _msg_1.content.msg.coolant = swap_message_uint16_t(_msg_1.content.msg.coolant);
    _msg_1.content.msg.tps = swap_message_uint16_t(_msg_1.content.msg.tps);
    _msg_1.content.msg.batteryvoltage = swap_message_uint16_t(_msg_1.content.msg.batteryvoltage);
    _msg_1.content.msg.afr1 = swap_message_uint16_t(_msg_1.content.msg.afr1);
    _msg_1.content.msg.afr2 = swap_message_uint16_t(_msg_1.content.msg.afr2);
    _msg_1.content.msg.barocorrection = swap_message_uint16_t(_msg_1.content.msg.barocorrection);
    _msg_1.content.msg.gammaenrich = swap_message_uint16_t(_msg_1.content.msg.gammaenrich);
    _msg_1.content.msg.ve1 = swap_message_uint16_t(_msg_1.content.msg.ve1);
    _msg_1.content.msg.cold_adv_deg = swap_message_uint16_t(_msg_1.content.msg.cold_adv_deg);
    _msg_1.content.msg.tpsdot = swap_message_uint16_t(_msg_1.content.msg.tpsdot);
    _msg_1.content.msg.mapdot = swap_message_uint16_t(_msg_1.content.msg.mapdot);
    _msg_1.content.msg.egov1 = swap_message_uint16_t(_msg_1.content.msg.egov1);
    _msg_1.content.msg.egov2 = swap_message_uint16_t(_msg_1.content.msg.egov2);
}
