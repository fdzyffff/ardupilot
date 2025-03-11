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

UAtt::UAtt()
{
    ;
}

// initialise
void UAtt::init()
{
    _ahrs_filter.set_cutoff_frequency(copter.g2.user_parameters.freq_sample.get(), copter.g2.user_parameters.freq_cutoff.get());
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
}

// update 
void UAtt::update()
{
    _ahrs_filter.apply(Vector3f(copter.ahrs_view->roll, copter.ahrs_view->pitch, copter.ahrs_view->yaw));
    roll = _ahrs_filter.get().x;
    pitch = _ahrs_filter.get().y;
    yaw = _ahrs_filter.get().z;
}

const Vector3f& UAtt::get_ahrs()
{
    return _ahrs_filter.get();
}