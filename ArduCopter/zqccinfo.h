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
  void update(float pixel_raw_x_in, float pixel_raw_y_in, float alt_cm_in);
  bool adjust_roll_pitch_yaw(float &roll, float &pitch, float angle_max, float &yaw_rate);
  bool adjust_climb_rate(float &target_climb_rate);
  void update_sonar_alt(float &target_climb_rate);
  void accumulate_lean(float roll, float pitch, float g_Dt);
  void release_lean(float &roll, float &pitch, float g_Dt);
  void reset_lean();

  float get_raw_x() {return _pixel_raw_x;}
  float get_raw_y() {return _pixel_raw_y;}
  float get_x() {return _pixel_x;}
  float get_y() {return _pixel_y;}
  float get_delta_climb_rate() {return _delta_climb_rate;}

  infoZQCC_class(void);

private:
  uint32_t last_update;
  float _pixel_raw_x;
  float _pixel_raw_y;
  float _pixel_x;
  float _pixel_y;
  float _delta_climb_rate;
  float _acc_roll;
  float _acc_pitch;
  bool _lean_running;
  bool _alt_avaliable;
  float _alt_cm_in;
  float _sonar_target_alt_cm;
  uint32_t _sonar_target_alt_update_ms;
};
