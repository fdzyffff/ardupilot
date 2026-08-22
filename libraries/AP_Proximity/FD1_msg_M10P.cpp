#include "FD1_msg_M10P.h"

#include <cstring>

FD1_msg_M10P::FD1_msg_M10P() :
    _state(ParseState::PREAMBLE1),
    _read(0),
    _expected_length(0),
    _accepted_length(0),
    _buffer{},
    _frame{},
    _updated(false)
{
}

bool FD1_msg_M10P::length_valid(uint16_t length) const
{
    return length >= FRAME_OVERHEAD &&
           length <= MAX_FRAME_LENGTH &&
           ((length - FRAME_OVERHEAD) % 2U) == 0;
}

void FD1_msg_M10P::reset_parser(uint8_t byte)
{
    _read = 0;
    _expected_length = 0;
    _state = ParseState::PREAMBLE1;
    if (byte == 0xA5) {
        _buffer[0] = byte;
        _read = 1;
        _state = ParseState::PREAMBLE2;
    }
}

void FD1_msg_M10P::parse(uint8_t byte)
{
    switch (_state) {
    case ParseState::PREAMBLE1:
        reset_parser(byte);
        break;
    case ParseState::PREAMBLE2:
        if (byte == 0x5A) {
            _buffer[1] = byte;
            _read = 2;
            _state = ParseState::DATA;
        } else {
            reset_parser(byte);
        }
        break;
    case ParseState::DATA:
        if (_read >= MAX_FRAME_LENGTH) {
            reset_parser(byte);
            break;
        }
        _buffer[_read++] = byte;
        if (_read == 4) {
            _expected_length = (uint16_t(_buffer[2]) << 8) | uint16_t(_buffer[3]);
            if (!length_valid(_expected_length)) {
                reset_parser(byte);
                break;
            }
        }
        if (_expected_length != 0 && _read == _expected_length) {
            if (_buffer[_expected_length - 2] == 0xFA && _buffer[_expected_length - 1] == 0xFB) {
                process_message();
            }
            reset_parser();
        }
        break;
    }
}

void FD1_msg_M10P::process_message()
{
    memset(_frame.data, 0, sizeof(_frame.data));
    memcpy(_frame.data, _buffer, _expected_length);
    _accepted_length = _expected_length;
    _updated = true;
}

bool FD1_msg_M10P::consume_frame()
{
    const bool updated = _updated;
    _updated = false;
    return updated;
}
