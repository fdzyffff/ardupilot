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

UGround::UGround()
{
    init();
}

// initialise
void UGround::init()
{
    _active = false;
    _last_update_ms = 0;
}

// clear return path and set home location.  This should be called as part of the arming procedure
void UGround::handle_info(bool valid, float input_x, float input_y)
{
    if (!valid) {return;}
    raw_info.x = input_x;
    raw_info.y = input_y;
    _last_update_ms = millis();
}

// update
void UGround::update()
{
    const uint32_t now = millis();
    uint32_t _time_out = (uint32_t)copter.g2.user_parameters.gcs_time_out.get();
    if ( _time_out!=0 && ((now - _last_update_ms) > _time_out) ) {
        _active = false;
    } else {
        _active = true;
    }
}


void Copter::Ugcs_state_update() 
{
    ;
}

void Copter::Ugcs_do_takeoff() // takeoff
{
    ;
}

void Copter::Ugcs_do_cruise() // fly and search
{
    ;
}

void Copter::Ugcs_do_lockon() // lock on target
{
    ;
}

void Copter::Ugcs_do_search() // stand by and look around
{
    ;
}

void Copter::Ugcs_do_attack() 
{
    ;
}