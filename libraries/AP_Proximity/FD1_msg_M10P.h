#pragma once

#include <FD_UART/FD1_message.h>

class FD1_msg_M10P : public FD1_message {
public:
    static constexpr uint16_t MAX_FRAME_LENGTH = 160;
    static constexpr uint16_t FRAME_OVERHEAD = 20;

    struct PACKED Frame {
        uint8_t data[MAX_FRAME_LENGTH];
    };

    FD1_msg_M10P();
    FD1_msg_M10P(const FD1_msg_M10P &other) = delete;
    FD1_msg_M10P &operator=(const FD1_msg_M10P&) = delete;

    void process_message() override;
    void parse(uint8_t byte) override;
    void swap_message() override {}

    bool consume_frame();
    const Frame &frame() const { return _frame; }
    uint16_t frame_length() const { return _accepted_length; }

private:
    enum class ParseState : uint8_t { PREAMBLE1, PREAMBLE2, DATA };
    void reset_parser(uint8_t byte = 0);
    bool length_valid(uint16_t length) const;

    ParseState _state;
    uint16_t _read;
    uint16_t _expected_length;
    uint16_t _accepted_length;
    uint8_t _buffer[MAX_FRAME_LENGTH];
    Frame _frame;
    bool _updated;
};

static_assert(sizeof(FD1_msg_M10P::Frame) == FD1_msg_M10P::MAX_FRAME_LENGTH, "M10P frame size");
