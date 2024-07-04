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
#pragma once

#include "NotifyDevice.h"

class Buzzer_uart: public NotifyDevice
{
public:
    /// Constructor
    Buzzer_uart() {}

    /// init - initialise the buzzer
    bool init(void) override;

    /// update - updates buzzer according to timed_updated.  Should be called at 50Hz
    void update() override;

private:

    /// on - turns the buzzer on or off
    void on(bool on_off);

    // musics - how many beeps will be played; read from
    // left-to-right, each bit represents 100ms
    const uint8_t    STABILIZE_BUZZ = 1;
    const uint8_t      ALTHOLD_BUZZ = 2;
    const uint8_t       LOITER_BUZZ = 3;
    const uint8_t         AUTO_BUZZ = 4;
    const uint8_t          RTL_BUZZ = 5;
    const uint8_t   ARM_FAILED_BUZZ = 6;
    const uint8_t          ARM_BUZZ = 7;
    const uint8_t  MODE_FAILED_BUZZ = 8;
    const uint8_t     LOW_VOLT_BUZZ = 9;
    const uint8_t          EKF_BUZZ = 10;
    const uint8_t        CRASH_BUZZ = 11;
    const uint8_t       DISARM_BUZZ = 12;
    const uint8_t PRE_ARM_GOOD_BUZZ = 13;
    const uint8_t      GPS_FIX_BUZZ = 14;
    const uint8_t      RTK_FIX_BUZZ = 15;

    void add_music(const uint8_t music, bool force_priority = true);
    /// play_music - plays the defined buzzer music
    void play_music(const uint8_t music);

    /// buzzer_flag_type - bitmask of current state and ap_notify states we track
    struct buzzer_flag_type {
        uint8_t on                  : 1;    // 1 if the buzzer is currently on
        uint8_t arming              : 1;    // 1 if we are beginning the arming process
        uint8_t armed               : 1;    // 0 = disarmed, 1 = armed
        uint8_t failsafe_battery    : 1;    // 1 if battery failsafe has triggered
        uint8_t ekf_bad             : 1;    // 1 if ekf position has gone bad
        uint8_t gyro_calibrated     : 1;    // 1 if calibrating gyro
        uint8_t pre_arm_check       : 1;    // 1 if pre-arm check has passed
        uint8_t gps_status;
    } _flags;

    uint8_t _music;
    uint8_t _next_music;
    uint32_t _music_start_time;

    AP_HAL::UARTDriver *_port;

    uint8_t _cmd_data[8];

    void update_playing_music();
    void update_music_to_play();

};
