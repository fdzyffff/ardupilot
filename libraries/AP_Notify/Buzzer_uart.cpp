/*
  Buzzer_uart driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Buzzer_uart.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

bool Buzzer_uart::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
    const AP_SerialManager &serial_manager = AP::serialmanager();
    if (!(_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_BUZZER, 0))) {
        _port = nullptr;
        return false;
    }

    on(false);
    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer_uart::update()
{
    update_playing_music();
    update_music_to_play();
}

void Buzzer_uart::update_music_to_play()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        add_music(ARM_FAILED_BUZZ);
        return;
    }

    if (AP_Notify::events.user_mode_change_failed) {
        add_music(MODE_FAILED_BUZZ);
    }

    if (AP_Notify::events.user_mode_change) {
        if (AP_Notify::flags.flight_mode == 0) {
            add_music(STABILIZE_BUZZ);
        }
        if (AP_Notify::flags.flight_mode == 2) {
            add_music(ALTHOLD_BUZZ);
        }
        if (AP_Notify::flags.flight_mode == 5) {
            add_music(LOITER_BUZZ);
        }
        if (AP_Notify::flags.flight_mode == 3) {
            add_music(AUTO_BUZZ);
        }
        if (AP_Notify::flags.flight_mode == 6) {
            add_music(RTL_BUZZ);
        }
    }
    // // initializing?
    // if (_flags.gyro_calibrated != AP_Notify::flags.gyro_calibrated) {
    //     _flags.gyro_calibrated = AP_Notify::flags.gyro_calibrated;
    //     add_music(INIT_GYRO);
    // }

    if (_flags.gps_status != AP_Notify::flags.gps_status) {
        _flags.gps_status = AP_Notify::flags.gps_status;
        if (_flags.gps_status == 3) {
            add_music(GPS_FIX_BUZZ);
        }
        if (_flags.gps_status == 5) {
            add_music(RTK_FIX_BUZZ);
        }
    }

    // check if prearm check are good
    if (AP_Notify::flags.pre_arm_check && !_flags.pre_arm_check) {
        _flags.pre_arm_check = true;
        add_music(PRE_ARM_GOOD_BUZZ);
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            add_music(ARM_BUZZ);
        } else {
            // single buzz when disarmed
            add_music(DISARM_BUZZ);
        }
        return;
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            add_music(EKF_BUZZ);
        }
        return;
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        add_music(CRASH_BUZZ, false);
        return;
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        add_music(LOW_VOLT_BUZZ, false);
        return;
    }
}


void Buzzer_uart::update_playing_music()
{
    if (_flags.on) {
        const uint32_t delta = AP_HAL::millis() - _music_start_time;
        if (delta >= 2000) {
            // finished playing music
            on(false);
            _music = _next_music;
            _next_music = 0;
        }
    } else {
        if (_music) {
            play_music(_music);
            on(true);
        } else {
            on(false);
        }
    }
}

// on - turns the buzzer on or off
void Buzzer_uart::on(bool turn_on)
{
    // return immediately if nothing to do
    if (_flags.on == turn_on) {
        return;
    }

    // update state
    _flags.on = turn_on;
}

void Buzzer_uart::add_music(const uint8_t music, bool force_priority)
{
    if (_music == 0) {
        _music = music;
    } else {
        if (force_priority) {
            _next_music = music;
        } else {
            if (_next_music == 0) {
                _next_music = music;
            }
        }
    }
}


/// play_music - plays the defined buzzer music
void Buzzer_uart::play_music(const uint8_t music)
{
    _music = music;
    _music_start_time = AP_HAL::millis();
    // switch (_music) {
    //     case 1:
    // } 
    _port->write(0x7E);
    _port->write(_music);
}
