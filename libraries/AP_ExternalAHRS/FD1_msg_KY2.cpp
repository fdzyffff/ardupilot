#include "FD1_msg_KY2.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;


FD1_msg_KY2::FD1_msg_KY2(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    _msg_1.length = FD1_MSG_KY2_FRAME_LEN;
}

void FD1_msg_KY2::parse(uint8_t temp)
{
    uint32_t now_ms = AP_HAL::millis();

    // Timeout: if gap > 5ms between bytes, reset state machine
    if (now_ms - _last_byte_ms > 5) {
        if (_msg.msg_state != FD1UART_msg_parser::FD1UART_PREAMBLE1) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            _msg.read = 0;
        }
    }
    _last_byte_ms = now_ms;

    switch (_msg.msg_state) {

    default:
    case FD1UART_msg_parser::FD1UART_PREAMBLE1:
        _msg.read = 0;
        _msg.data[0] = temp;
        _msg.data_length = 0;
        if (temp == PREAMBLE1) {
            _msg.read = 1;
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
        }
        break;

    case FD1UART_msg_parser::FD1UART_PREAMBLE2:
        _msg.data[_msg.read] = temp;
        _msg.read++;
        if (temp == PREAMBLE2) {
            _msg.data_length = 10;   // initial safe value, overwritten at bytes 4-5
            _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
        } else {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;

    case FD1UART_msg_parser::FD1UART_DATA:
        _msg.data[_msg.read] = temp;
        _msg.read++;

        // Check frame ID at byte position 4 (just stored bytes 2-3)
        if (_msg.read == 4) {
            uint16_t frame_id = (uint16_t)_msg.data[2] | ((uint16_t)_msg.data[3] << 8);
            // gcs().send_text(MAV_SEVERITY_INFO, "_msg.read: %d, frame_id: %#x", _msg.read, frame_id);
            if (frame_id != FRAME_ID) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
        }

        // Read frame data length at byte position 6 (just stored bytes 4-5)
        if (_msg.read == 6) {
            _msg.data_length = (uint16_t)_msg.data[4] | ((uint16_t)_msg.data[5] << 8);

            // Sanity check on data length
            if (_msg.data_length == 0 || _msg.data_length > FD1_MSG_KY2_DATA_LEN) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
        }

        // Check if we've read all data bytes
        if (_msg.read >= (FD1_MSG_KY2_OVERHEAD + _msg.data_length)) {
            _msg.msg_state = FD1UART_msg_parser::FD1UART_CRC;
        }
        break;

    case FD1UART_msg_parser::FD1UART_CRC: {
        _msg.data[_msg.read] = temp;
        _msg.read++;

        uint16_t crc_offset = FD1_MSG_KY2_OVERHEAD + _msg.data_length;
        if (_msg.read >= (crc_offset + FD1_MSG_KY2_CRC_LEN)) {
            // Compute CRC32 over header+ID+length+data (initial value = 1)
            uint32_t computed_crc = crc32_compute(_msg.data, crc_offset);

            // Received CRC is little-endian
            uint32_t received_crc = (uint32_t)_msg.data[crc_offset]
                                  | ((uint32_t)_msg.data[crc_offset + 1] << 8)
                                  | ((uint32_t)_msg.data[crc_offset + 2] << 16)
                                  | ((uint32_t)_msg.data[crc_offset + 3] << 24);

            // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %#x - %#x",_msg.msg_state, computed_crc, received_crc);
            if (computed_crc == received_crc) {
                process_message();
            }

            _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
        }
        break;
    }

    } // switch
}

uint32_t FD1_msg_KY2::crc32_compute(const uint8_t *buf, uint32_t size) const
{
    uint32_t crc = 1;
    for (uint32_t i = 0; i < size; i++) {
        crc = _crc32_tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    }
    return crc;
}

void FD1_msg_KY2::process_message(void)
{
    uint16_t total_len = FD1_MSG_KY2_OVERHEAD + _msg.data_length + FD1_MSG_KY2_CRC_LEN;
    if (total_len > FD1_MSG_KY2_FRAME_LEN) {
        total_len = FD1_MSG_KY2_FRAME_LEN;
    }
    for (uint16_t i = 0; i < total_len; i++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    _msg_1.length = total_len;
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}
