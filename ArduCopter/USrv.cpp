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

USrv::USrv()
{
    init();
}

// initialise
void USrv::init()
{
    // _target_loc = plane.current_loc;
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.count = 0;
}

void USrv::update()
{
    float yaw_thrust = constrain_float(copter.motors->get_yaw() + copter.motors->get_yaw_ff(), -1.0f, 1.0f);
    float roll_thrust = constrain_float(srv_in.roll, -1.0f, 1.0f);
    float pitch_thrust = constrain_float(srv_in.pitch, -1.0f, 1.0f);

    srv_out.smotor1 = constrain_float(0.2f*yaw_thrust + 0.8f*roll_thrust, -1.0f, 1.0f);
    srv_out.smotor2 = constrain_float(0.2f*yaw_thrust - 0.8f*roll_thrust, -1.0f, 1.0f);
    srv_out.smotor3 = constrain_float(0.2f*yaw_thrust - 0.8f*pitch_thrust, -1.0f, 1.0f);
    srv_out.smotor4 = constrain_float(0.2f*yaw_thrust + 0.8f*pitch_thrust, -1.0f, 1.0f);

    SRV_Channels::set_output_scaled(SRV_Channel::k_smotor1, srv_out.smotor1);
    SRV_Channels::set_output_scaled(SRV_Channel::k_smotor2, srv_out.smotor2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_smotor3, srv_out.smotor3);
    SRV_Channels::set_output_scaled(SRV_Channel::k_smotor4, srv_out.smotor4);
}

void USrv::print() {
    // gcs().send_text(MAV_SEVERITY_INFO, "[mlstate.vehicle_mode %d]", (uint8_t)mlstate.vehicle_mode);
}
