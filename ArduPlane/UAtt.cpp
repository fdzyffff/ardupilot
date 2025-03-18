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


#include "Plane.h"

UAtt::UAtt()
{
    ;
}

// initialise
void UAtt::init()
{
    _ahrs_filter.set_cutoff_frequency(400.f, 200.f);
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
}

// update 
void UAtt::update()
{
    _ahrs_filter.apply(Vector3f(plane.ahrs.get_roll(), plane.ahrs.get_pitch(), plane.ahrs.get_yaw()));
    roll = _ahrs_filter.get().x;
    pitch = _ahrs_filter.get().y;
    yaw = _ahrs_filter.get().z;
    // roll = plane.ahrs.get_roll();
    // pitch = plane.ahrs.get_pitch();
    // yaw = plane.ahrs.get_yaw();
}

const Vector3f& UAtt::get_ahrs()
{
    return _ahrs_filter.get();
}