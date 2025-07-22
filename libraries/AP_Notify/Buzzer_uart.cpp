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
#include <GCS_MAVLink/GCS.h>

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

    reset_music();

    _print_test = false;

    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer_uart::update()
{
    update_playing_music();
    update_music_to_play();
}

void Buzzer_uart::reset_music()
{
    for (uint8_t i_music = 0; i_music < MAX_BUZZER_MUSIC_NUM; i_music++) {
        _music_event_buffer[i_music].music = 0;
        _music_event_buffer[i_music].time = 0;
        _music_loop_buffer[i_music].music = 0;
        _music_loop_buffer[i_music].time = 0;
    }
}

void Buzzer_uart::update_music_to_play()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        add_event_music(ARM_FAILED_BUZZ);
        if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "ARM_FAILED_BUZZ");}
        return;
    }

    if (AP_Notify::events.user_mode_change_failed) {
        add_event_music(MODE_FAILED_BUZZ);
        if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "MODE_FAILED_BUZZ");}
    }

    if (AP_Notify::events.user_mode_change) {
        if (AP_Notify::flags.flight_mode == 0) {
            add_event_music(STABILIZE_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "STABILIZE_BUZZ");}
        }
        if (AP_Notify::flags.flight_mode == 2) {
            add_event_music(ALTHOLD_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "ALTHOLD_BUZZ");}
        }
        if (AP_Notify::flags.flight_mode == 5) {
            add_event_music(LOITER_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "LOITER_BUZZ");}
        }
        if (AP_Notify::flags.flight_mode == 3) {
            add_event_music(AUTO_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "AUTO_BUZZ");}
        }
        if (AP_Notify::flags.flight_mode == 6) {
            add_event_music(RTL_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "RTL_BUZZ");}
        }
    }
    // // initializing?
    // if (_flags.gyro_calibrated != AP_Notify::flags.gyro_calibrated) {
    //     _flags.gyro_calibrated = AP_Notify::flags.gyro_calibrated;
    //     add_music(INIT_GYRO);
    // }

    if (_flags.gps_status != AP_Notify::flags.gps_status) {
        _flags.gps_status = AP_Notify::flags.gps_status;
        if (_flags.gps_status == 3 || _flags.gps_status == 4) {
            add_event_music(GPS_FIX_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "GPS_FIX_BUZZ");}
        }
        if (_flags.gps_status == 5 || _flags.gps_status == 6) {
            add_event_music(RTK_FIX_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "RTK_FIX_BUZZ");}
        }
    }

    // check if prearm check are good
    if (AP_Notify::flags.pre_arm_check && !_flags.pre_arm_check) {
        if (AP_HAL::millis() - _pre_arm_check_time > 20000) {
            _flags.pre_arm_check = true;
            add_event_music(PRE_ARM_GOOD_BUZZ, 5);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "PRE_ARM_GOOD_BUZZ");}
        }
    } else {
        _pre_arm_check_time = AP_HAL::millis();
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            add_event_music(ARM_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "ARM_BUZZ");}
        } else {
            // single buzz when disarmed
            add_event_music(DISARM_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "DISARM_BUZZ");}
        }
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            add_loop_music(EKF_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "EKF_BUZZ");}
        } else {
            remove_loop_music(EKF_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "EKF_BUZZ remove");}
        }
    }

    // if vehicle lost was enabled, starting beep
    if (_flags.vehicle_lost != AP_Notify::flags.vehicle_lost) {
        _flags.vehicle_lost = AP_Notify::flags.vehicle_lost;
        if (_flags.vehicle_lost) {
            // ekf bad warning buzz
            add_loop_music(CRASH_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "CRASH_BUZZ");}
        } else {
            remove_loop_music(CRASH_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "EKF_BUZZ remove");}
        }
    }

    // if battery failsafe constantly single buzz
    if (_flags.failsafe_battery != AP_Notify::flags.failsafe_battery) {
        _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
        if (_flags.failsafe_battery) {
            // ekf bad warning buzz
            add_loop_music(LOW_VOLT_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "LOW_VOLT_BUZZ");}
            gcs().send_text(MAV_SEVERITY_INFO, "LOW_VOLT_BUZZ");
        } else {
            remove_loop_music(LOW_VOLT_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "LOW_VOLT_BUZZ remove");}
        }
    }

    // if ekf switched automatically due to bad GPS
    if (_flags.ekf_switch != AP_Notify::flags.ekf_switch) {
        _flags.ekf_switch = AP_Notify::flags.ekf_switch;
        if (_flags.ekf_switch) {
            // ekf bad warning buzz
            add_loop_music(EKF_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "EKF_BUZZ");}
        } else {
            remove_loop_music(EKF_BUZZ);
            if (_print_test) {gcs().send_text(MAV_SEVERITY_INFO, "EKF_BUZZ remove");}
        }
    }

}

void Buzzer_uart::update_playing_music()
{
    if (_flags.on) {
        if ((AP_HAL::millis() - _music_start_time)/1000 > _current_music.time) {
            on(false);
        }
    }

    if (!_flags.on) {
        if (_flags.loop) {
            _flags.loop = 0;
            play_loop_music();
        } else {
            play_event_music();
        }
    }
}

void Buzzer_uart::play_event_music() 
{
    if (_music_event_buffer[0].music > 0) {
        _current_music.music = _music_event_buffer[_i_music_loop].music;
        _current_music.time = _music_event_buffer[_i_music_loop].time;
        play_music();

        _music_event_buffer[0].music = 0;
        _music_event_buffer[0].time = 0;
        // remove played music
        for (uint8_t i_music = 1; i_music < MAX_BUZZER_MUSIC_NUM; i_music++) {
            if (_music_event_buffer[i_music].music > 0) {
                _music_event_buffer[i_music-1].music = _music_event_buffer[i_music].music;
                _music_event_buffer[i_music-1].time = _music_event_buffer[i_music].time;
            } else {
                break;
            }
        }
    }
}

void Buzzer_uart::play_loop_music()
{
    uint8_t i_max = 0;
    // check number of music stored in loop buffer
    for (i_max = 0; i_max < MAX_BUZZER_MUSIC_NUM; i_max++) {
        if (_music_loop_buffer[i_max].music > 0) {
            ;
        } else {
            break;
        }
    }
    if (_i_music_loop >= i_max) {
        _i_music_loop = 0;
    }
    if (_music_loop_buffer[_i_music_loop].music > 0) {
        _current_music.music = _music_loop_buffer[_i_music_loop].music;
        _current_music.time = _music_loop_buffer[_i_music_loop].time;
        play_music();
        // move to next loop music
        _i_music_loop++;
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

void Buzzer_uart::add_event_music(const uint8_t music, uint8_t time)
{
    uint8_t i_max = 0;
    // check number of music stored in loop buffer
    for (i_max = 0; i_max < MAX_BUZZER_MUSIC_NUM; i_max++) {
        if (_music_event_buffer[i_max].music > 0) {
            // jump exsited music
            if (_music_event_buffer[i_max].music == music) {
                return;
            }
        } else {
            break;
        }
    }

    if (i_max < 8) {
        _music_event_buffer[i_max].music = music;
        _music_event_buffer[i_max].time = time;
    }
}
    
void Buzzer_uart::add_loop_music(const uint8_t music, uint8_t time)
{
    uint8_t i_max = 0;
    // check number of music stored in loop buffer
    for (i_max = 0; i_max < MAX_BUZZER_MUSIC_NUM; i_max++) {
        if (_music_loop_buffer[i_max].music > 0) {
            // jump exsited music
            if (_music_loop_buffer[i_max].music == music) {
                return;
            }
        } else {
            break;
        }
    }

    if (i_max < 8) {
        _music_loop_buffer[i_max].music = music;
        _music_loop_buffer[i_max].time = time;
    }
}

void Buzzer_uart::remove_loop_music(const uint8_t music)
{
    uint8_t i_music = 0;
    for (i_music = 0; i_music < MAX_BUZZER_MUSIC_NUM; i_music++) {
        if (_music_loop_buffer[i_music].music  == music) {
            _music_loop_buffer[i_music].music = 0;
            _music_loop_buffer[i_music].time = 0;
            // remove this music
            for (uint8_t j_music = i_music+1; j_music < MAX_BUZZER_MUSIC_NUM; j_music++) {
                if (_music_event_buffer[j_music].music > 0) {
                    _music_event_buffer[j_music-1].music = _music_event_buffer[j_music].music;
                    _music_event_buffer[j_music-1].time = _music_event_buffer[j_music].time;
                } else {
                    break;
                }
            }
        } else {
            break;
        }
    }
}

/// play_music - plays the defined buzzer music
void Buzzer_uart::play_music()
{
    _music_start_time = AP_HAL::millis();
    // switch (_music) {
    //     case 1:
    // } 

    _cmd_data[0] = 0x7E;
    _cmd_data[1] = 0xFF;
    _cmd_data[2] = 0x06;
    _cmd_data[3] = 0x03;
    _cmd_data[4] = 0x00;
    _cmd_data[5] = 0x00;
    _cmd_data[6] = _current_music.music;
    _cmd_data[7] = 0xEF;
    _port->write(_cmd_data, sizeof(_cmd_data));
}
