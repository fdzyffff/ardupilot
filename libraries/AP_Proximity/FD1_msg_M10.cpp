#include "FD1_msg_M10.h"

#include <cstring>

FD1_msg_M10::FD1_msg_M10() :
    _state(ParseState::PREAMBLE1),
    _read(0),
    _buffer{},
    _frame{},
    _updated(false)
{
}

void FD1_msg_M10::reset_parser(uint8_t byte)
{
    _read = 0;
    _state = ParseState::PREAMBLE1;
    if (byte == 0xA5) {
        _buffer[0] = byte;
        _read = 1;
        _state = ParseState::PREAMBLE2;
    }
}

void FD1_msg_M10::parse(uint8_t byte)
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
        if (_read >= FRAME_LENGTH) {
            reset_parser(byte);
            break;
        }
        _buffer[_read++] = byte;
        if (_read == FRAME_LENGTH) {
            if (_buffer[FRAME_LENGTH - 2] == 0xFA && _buffer[FRAME_LENGTH - 1] == 0xFB) {
                process_message();
            }
            reset_parser();
        }
        break;
    }
}

void FD1_msg_M10::process_message()
{
    memcpy(&_frame, _buffer, FRAME_LENGTH);
    _updated = true;
}

bool FD1_msg_M10::consume_frame()
{
    const bool updated = _updated;
    _updated = false;
    return updated;
}
