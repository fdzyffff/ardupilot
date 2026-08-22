/*
  UART music player backend
*/
#include "Buzzer_uart.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_Notify.h"

bool Buzzer_uart::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_BUZZER, 0);
    if (_port == nullptr) {
        return false;
    }

    _armed = AP_Notify::flags.armed;
    _failsafe_battery = AP_Notify::flags.failsafe_battery;
    _ekf_bad = AP_Notify::flags.ekf_bad;
    _vehicle_lost = AP_Notify::flags.vehicle_lost;
    _gps_status = AP_Notify::flags.gps_status;
    _pre_arm_good_start_ms = AP_HAL::millis();
    return true;
}

void Buzzer_uart::update()
{
    update_playing_music();
    update_music_to_play();
    play_next_music();
}

void Buzzer_uart::update_music_to_play()
{
    if (AP_Notify::events.arming_failed) {
        add_event_music(ARM_FAILED_BUZZ);
    }
    if (AP_Notify::events.user_mode_change_failed) {
        add_event_music(MODE_FAILED_BUZZ);
    }
    if (AP_Notify::events.user_mode_change) {
        switch (AP_Notify::flags.flight_mode) {
        case 0:
            add_event_music(STABILIZE_BUZZ);
            break;
        case 2:
            add_event_music(ALTHOLD_BUZZ);
            break;
        case 3:
            add_event_music(AUTO_BUZZ);
            break;
        case 5:
            add_event_music(LOITER_BUZZ);
            break;
        case 6:
            add_event_music(RTL_BUZZ);
            break;
        default:
            break;
        }
    }

    if (_gps_status != AP_Notify::flags.gps_status) {
        _gps_status = AP_Notify::flags.gps_status;
        if ((_gps_status == 3) || (_gps_status == 4)) {
            add_event_music(GPS_FIX_BUZZ);
        } else if ((_gps_status == 5) || (_gps_status == 6)) {
            add_event_music(RTK_FIX_BUZZ);
        }
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (!AP_Notify::flags.pre_arm_check) {
        _pre_arm_good_start_ms = now_ms;
        _pre_arm_announced = false;
    } else if (!_pre_arm_announced && (now_ms - _pre_arm_good_start_ms >= 20000U)) {
        _pre_arm_announced = true;
        add_event_music(PRE_ARM_GOOD_BUZZ, 5);
    }

    if (_armed != AP_Notify::flags.armed) {
        _armed = AP_Notify::flags.armed;
        add_event_music(_armed ? ARM_BUZZ : DISARM_BUZZ);
    }

    if (_ekf_bad != AP_Notify::flags.ekf_bad) {
        _ekf_bad = AP_Notify::flags.ekf_bad;
        if (_ekf_bad) {
            add_loop_music(EKF_BUZZ);
        } else {
            remove_loop_music(EKF_BUZZ);
        }
    }

    if (_vehicle_lost != AP_Notify::flags.vehicle_lost) {
        _vehicle_lost = AP_Notify::flags.vehicle_lost;
        if (_vehicle_lost) {
            add_loop_music(CRASH_BUZZ);
        } else {
            remove_loop_music(CRASH_BUZZ);
        }
    }

    if (_failsafe_battery != AP_Notify::flags.failsafe_battery) {
        _failsafe_battery = AP_Notify::flags.failsafe_battery;
        if (_failsafe_battery) {
            add_loop_music(LOW_VOLT_BUZZ);
        } else {
            remove_loop_music(LOW_VOLT_BUZZ);
        }
    }
}

void Buzzer_uart::update_playing_music()
{
    if (_playing && (AP_HAL::millis() - _music_start_ms >= uint32_t(_current_music.duration_s) * 1000U)) {
        _playing = false;
        _current_music = {};
    }
}

void Buzzer_uart::play_next_music()
{
    if (_playing) {
        return;
    }

    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        if (_play_loop_next) {
            uint8_t loop_count = 0;
            while ((loop_count < MAX_MUSIC_NUM) && (_loop_buffer[loop_count].music != 0)) {
                loop_count++;
            }
            if (loop_count != 0) {
                if (_loop_index >= loop_count) {
                    _loop_index = 0;
                }
                const MusicItem item = _loop_buffer[_loop_index++];
                _play_loop_next = false;
                if (play_music(item)) {
                    return;
                }
            }
            _play_loop_next = false;
        } else if (_event_buffer[0].music != 0) {
            const MusicItem item = _event_buffer[0];
            _event_buffer[0] = {};
            compact_buffer(_event_buffer);
            _play_loop_next = true;
            if (play_music(item)) {
                return;
            }
        } else {
            _play_loop_next = true;
        }
    }
}

bool Buzzer_uart::play_music(const MusicItem &item)
{
    if ((_port == nullptr) || (item.music == 0)) {
        return false;
    }

    const uint8_t command[8] = {0x7E, 0xFF, 0x06, 0x03, 0x10, 0x00, item.music, 0xEF};
    if (_port->txspace() < sizeof(command)) {
        return false;
    }
    const size_t written = _port->write(command, sizeof(command));
    if (written != sizeof(command)) {
        return false;
    }

    _current_music = item;
    _music_start_ms = AP_HAL::millis();
    _playing = true;
    return true;
}

void Buzzer_uart::add_event_music(uint8_t music, uint8_t duration_s)
{
    for (uint8_t i = 0; i < MAX_MUSIC_NUM; i++) {
        if (_event_buffer[i].music == music) {
            return;
        }
        if (_event_buffer[i].music == 0) {
            _event_buffer[i] = {duration_s, music};
            return;
        }
    }
}

void Buzzer_uart::add_loop_music(uint8_t music, uint8_t duration_s)
{
    for (uint8_t i = 0; i < MAX_MUSIC_NUM; i++) {
        if (_loop_buffer[i].music == music) {
            return;
        }
        if (_loop_buffer[i].music == 0) {
            _loop_buffer[i] = {duration_s, music};
            return;
        }
    }
}

void Buzzer_uart::remove_loop_music(uint8_t music)
{
    for (uint8_t i = 0; i < MAX_MUSIC_NUM; i++) {
        if (_loop_buffer[i].music == music) {
            _loop_buffer[i] = {};
            compact_buffer(_loop_buffer);
            if (_loop_index > i) {
                _loop_index--;
            }
            return;
        }
    }
}

void Buzzer_uart::compact_buffer(MusicItem (&buffer)[MAX_MUSIC_NUM])
{
    uint8_t write_index = 0;
    for (uint8_t read_index = 0; read_index < MAX_MUSIC_NUM; read_index++) {
        if (buffer[read_index].music != 0) {
            if (write_index != read_index) {
                buffer[write_index] = buffer[read_index];
                buffer[read_index] = {};
            }
            write_index++;
        }
    }
}
