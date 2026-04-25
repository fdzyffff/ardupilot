#pragma once

#include <AP_HAL/AP_HAL.h>
#include <string.h>

#define YOLO_MAX_DETECTIONS   5
#define YOLO_DETECTION_SIZE   21
#define YOLO_MAX_DATA_SIZE    (YOLO_MAX_DETECTIONS * YOLO_DETECTION_SIZE)
#define YOLO_MAX_FRAME_BUF    (3 + YOLO_MAX_DATA_SIZE + 2)

struct PACKED YoloDetection {
    uint8_t  class_id;
    float    confidence;
    float    offset_x;
    float    offset_y;
    float    norm_width;
    float    norm_height;
};

struct YoloFrame {
    uint8_t        num_detections;
    YoloDetection  detections[YOLO_MAX_DETECTIONS];
    bool           updated;
    uint32_t       timestamp_ms;
};

class FD_msg_YOLO {
public:
    FD_msg_YOLO();

    FD_msg_YOLO(const FD_msg_YOLO &other) = delete;
    FD_msg_YOLO &operator=(const FD_msg_YOLO&) = delete;

    void parse(uint8_t byte);

    bool enable() const { return _enable; }
    void set_enable()   { _enable = true; }

    YoloFrame frame;
    uint32_t  parse_count;

private:
    static const uint8_t HEADER1 = 0xA5;
    static const uint8_t HEADER2 = 0x5A;
    static const uint8_t END_FLAG = 0xFF;

    enum class ParseState : uint8_t {
        WAIT_HEADER1,
        WAIT_HEADER2,
        WAIT_COUNT,
        WAIT_DATA,
        WAIT_CHECKSUM,
        WAIT_END,
    };

    ParseState _state;
    uint8_t  _data_buf[YOLO_MAX_DATA_SIZE];
    uint8_t  _expected_count;
    uint16_t _expected_data_len;
    uint16_t _data_idx;
    uint8_t  _checksum;
    bool     _enable;

    void process_frame();
};
