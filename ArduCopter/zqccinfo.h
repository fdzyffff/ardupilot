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


class infoZQCC_class
{

public:
  void init();
  bool running();
  void update(float pixel_raw_x_in, float pixel_raw_y_in);
  bool adjust_roll_pitch(float &roll, float &pitch, float angle_max);

  infoZQCC_class(void);

private:
  uint32_t last_update;
  float _pixel_raw_x;
  float _pixel_raw_y;
  float _pixel_x;
  float _pixel_y;
};
