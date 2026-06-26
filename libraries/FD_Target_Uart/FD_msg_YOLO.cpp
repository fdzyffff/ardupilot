#include "FD_msg_YOLO.h"
#include <AP_HAL/AP_HAL.h>

FD_msg_YOLO::FD_msg_YOLO()
    : _state(ParseState::WAIT_HEADER1)
    , _expected_count(0)
    , _expected_data_len(0)
    , _data_idx(0)
    , _checksum(0)
    , _enable(false)
{
    frame.num_detections = 0;
    frame.updated = false;
    frame.timestamp_ms = 0;
    parse_count = 0;
}

void FD_msg_YOLO::parse(uint8_t byte)
{
    switch (_state) {
    case ParseState::WAIT_HEADER1:
        if (byte == HEADER1) {
            _state = ParseState::WAIT_HEADER2;
        }
        break;

    case ParseState::WAIT_HEADER2:
        if (byte == HEADER2) {
            _state = ParseState::WAIT_COUNT;
        } else {
            _state = ParseState::WAIT_HEADER1;
        }
        break;

    case ParseState::WAIT_COUNT:
        _expected_count = byte;
        _checksum = byte;
        if (_expected_count > YOLO_MAX_DETECTIONS) {
            _state = ParseState::WAIT_HEADER1;
            break;
        }
        _expected_data_len = (uint16_t)_expected_count * YOLO_DETECTION_SIZE;
        _data_idx = 0;
        if (_expected_data_len == 0) {
            _state = ParseState::WAIT_CHECKSUM;
        } else {
            _state = ParseState::WAIT_DATA;
        }
        break;

    case ParseState::WAIT_DATA:
        _data_buf[_data_idx++] = byte;
        _checksum += byte;
        if (_data_idx >= _expected_data_len) {
            _state = ParseState::WAIT_CHECKSUM;
        }
        break;

    case ParseState::WAIT_CHECKSUM:
        if (byte == (_checksum & 0xFF)) {
            _state = ParseState::WAIT_END;
        } else {
            _state = ParseState::WAIT_HEADER1;
        }
        break;

    case ParseState::WAIT_END:
        if (byte == END_FLAG) {
            process_frame();
        }
        _state = ParseState::WAIT_HEADER1;
        break;
    }
}

void FD_msg_YOLO::process_frame()
{
    frame.num_detections = _expected_count;
    frame.timestamp_ms = AP_HAL::millis();

    for (uint8_t i = 0; i < _expected_count; i++) {
        const uint8_t *p = &_data_buf[i * YOLO_DETECTION_SIZE];
        YoloDetection &det = frame.detections[i];
        det.class_id = p[0];
        memcpy(&det.confidence,  &p[1],  sizeof(float));
        memcpy(&det.offset_x,   &p[5],  sizeof(float));
        memcpy(&det.offset_y,   &p[9],  sizeof(float));
        memcpy(&det.norm_width,  &p[13], sizeof(float));
        memcpy(&det.norm_height, &p[17], sizeof(float));
    }

    frame.updated = true;
    parse_count++;
}
