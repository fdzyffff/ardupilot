#pragma once

#include <FD_UART/FD1_message.h>

class FD1_msg_M10 : public FD1_message {
public:
    static constexpr uint16_t FRAME_LENGTH = 102;

    struct PACKED Frame {
        uint8_t header[2];
        uint8_t angle_be[2];
        uint8_t speed_be[2];
        uint8_t distance_be[84];
        uint8_t gps_time[10];
        uint8_t footer[2];
    };

    FD1_msg_M10();
    FD1_msg_M10(const FD1_msg_M10 &other) = delete;
    FD1_msg_M10 &operator=(const FD1_msg_M10&) = delete;

    void process_message() override;
    void parse(uint8_t byte) override;
    void swap_message() override {}

    bool consume_frame();
    const Frame &frame() const { return _frame; }

private:
    enum class ParseState : uint8_t { PREAMBLE1, PREAMBLE2, DATA };
    void reset_parser(uint8_t byte = 0);

    ParseState _state;
    uint16_t _read;
    uint8_t _buffer[FRAME_LENGTH];
    Frame _frame;
    bool _updated;
};

static_assert(sizeof(FD1_msg_M10::Frame) == FD1_msg_M10::FRAME_LENGTH, "M10 frame size");
