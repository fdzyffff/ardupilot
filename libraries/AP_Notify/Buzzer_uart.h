/*
  UART music player backend
*/
#pragma once

#include "NotifyDevice.h"

class Buzzer_uart : public NotifyDevice
{
public:
    Buzzer_uart() = default;

    bool init() override;
    void update() override;

private:
    static constexpr uint8_t MAX_MUSIC_NUM = 8;

    enum Music : uint8_t {
        STABILIZE_BUZZ = 1,
        ALTHOLD_BUZZ = 2,
        LOITER_BUZZ = 3,
        AUTO_BUZZ = 4,
        RTL_BUZZ = 5,
        ARM_FAILED_BUZZ = 6,
        ARM_BUZZ = 7,
        MODE_FAILED_BUZZ = 8,
        LOW_VOLT_BUZZ = 9,
        EKF_BUZZ = 10,
        CRASH_BUZZ = 11,
        DISARM_BUZZ = 12,
        PRE_ARM_GOOD_BUZZ = 13,
        GPS_FIX_BUZZ = 14,
        RTK_FIX_BUZZ = 15,
    };

    struct MusicItem {
        uint8_t duration_s;
        uint8_t music;
    };

    void add_event_music(uint8_t music, uint8_t duration_s = 2);
    void add_loop_music(uint8_t music, uint8_t duration_s = 2);
    void remove_loop_music(uint8_t music);
    void update_playing_music();
    void update_music_to_play();
    void play_next_music();
    bool play_music(const MusicItem &item);
    static void compact_buffer(MusicItem (&buffer)[MAX_MUSIC_NUM]);

    AP_HAL::UARTDriver *_port = nullptr;
    MusicItem _event_buffer[MAX_MUSIC_NUM]{};
    MusicItem _loop_buffer[MAX_MUSIC_NUM]{};
    MusicItem _current_music{};

    uint32_t _music_start_ms = 0;
    uint32_t _pre_arm_good_start_ms = 0;
    uint8_t _loop_index = 0;
    uint8_t _gps_status = 0;
    bool _playing = false;
    bool _play_loop_next = false;
    bool _armed = false;
    bool _failsafe_battery = false;
    bool _ekf_bad = false;
    bool _vehicle_lost = false;
    bool _pre_arm_announced = false;
};
