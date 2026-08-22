#include "FD1_msg_N10P.h"

#include <cstring>

FD1_msg_N10P::FD1_msg_N10P() :
    _state(ParseState::PREAMBLE1),
    _read(0),
    _checksum(0),
    _buffer{},
    _frame{},
    _updated(false)
{
}

void FD1_msg_N10P::reset_parser(uint8_t byte)
{
    _read = 0;
    _checksum = 0;
    _state = ParseState::PREAMBLE1;
    if (byte == 0xA5) {
        _buffer[0] = byte;
        _checksum = byte;
        _read = 1;
        _state = ParseState::PREAMBLE2;
    }
}

void FD1_msg_N10P::parse(uint8_t byte)
{
    switch (_state) {
    case ParseState::PREAMBLE1:
        reset_parser(byte);
        break;
    case ParseState::PREAMBLE2:
        if (byte == 0x5A) {
            _buffer[1] = byte;
            _checksum = uint8_t(_checksum + byte);
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
        if (_read == 2 && byte != FRAME_LENGTH) {
            reset_parser(byte);
            break;
        }
        _buffer[_read] = byte;
        if (_read == FRAME_LENGTH - 1) {
            if (byte == _checksum) {
                process_message();
            }
            reset_parser();
        } else {
            _checksum = uint8_t(_checksum + byte);
            _read++;
        }
        break;
    }
}

void FD1_msg_N10P::process_message()
{
    memcpy(&_frame, _buffer, FRAME_LENGTH);
    _updated = true;
}

bool FD1_msg_N10P::consume_frame()
{
    const bool updated = _updated;
    _updated = false;
    return updated;
}
