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

#include "Copter.h"
//#include "UCam.h"

UCam::UCam()
{    init();
}

// initialise
void UCam::init()
{
    _active = false;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    _last_update_ms = 0;
}

// clear return path and set home location.  This should be called as part of the arming procedure
void UCam::handle_info(float input_x, float input_y, bool valid)
{
    if (!valid) {return;}
    raw_info.x = input_x;
    raw_info.y = input_y;
    _last_update_ms = millis();
}

// update
void UCam::update()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.cam_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _active = false;
    } else {
        _active = true;
    }
}
