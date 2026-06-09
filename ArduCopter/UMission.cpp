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

UMission::UMission()
{
    ;
}

// initialise
void UMission::init()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "UMission init");
}

void UMission::update()
{
    check_alive();
    update_log();
}

void UMission::check_alive()
{
    if (_last_target_update_ms < 30000) {return;}
    if (millis() - _last_target_update_ms > 20000) {
        if (_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Mission Target lost");
        }
        _alive = false;
    } else {
        if (!_alive) {
            gcs().send_text(MAV_SEVERITY_INFO, "Mission Target recieve");
        }
        _alive = true;
    }
}

void UMission::update_log()
{
    if (!_alive) {return;}
    uint32_t now_ms = millis();
    if (now_ms - _last_log_ms < 500) {return;}

    _last_log_ms = now_ms;

    // AP::logger().WriteStreaming("UWGT",
    //                             "TimeUS,FRONT,LEFT,RIGHT",
    //                             "s---",
    //                             "F---",
    //                             "Qfff",
    //                             AP_HAL::micros64(),
    //                             (float)hxts_hy_weight_packet.Front,
    //                             (float)hxts_hy_weight_packet.LEFT,
    //                             (float)hxts_hy_weight_packet.RIGHT);
}

void UMission::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_JKY_TARGET_MSG) {
        mavlink_jky_target_msg_t packet;
        mavlink_msg_jky_target_msg_decode(&msg, &packet);
        Location tmp_loc;
        tmp_loc.lat = packet.latitude;
        tmp_loc.lng = packet.longitude;
        tmp_loc.set_alt_cm(packet.altitude, Location::AltFrame::ABOVE_HOME);
        gcs().send_text(MAV_SEVERITY_INFO, "%d, %d, %d", int(tmp_loc.lat), int(tmp_loc.lng), int(tmp_loc.alt));
        set_target_loc(tmp_loc);
    }
}

void UMission::set_target_loc(Location& loc_in)
{
    Vector3f temp_pos;
    Vector3f target_pos;
    if (loc_in.get_vector_from_origin_NEU(temp_pos)) {
        if (millis() - _last_target_update_ms > 30000) {
            _target_pos.reset(temp_pos);
        } else {
            float dt = (float)(millis() - _last_target_update_ms) * 0.001f;
            _target_pos.apply(temp_pos, dt);
        }
        _target_loc = Location(_target_pos.get(), Location::AltFrame::ABSOLUTE);
        _last_target_update_ms = millis();


        // printf("temp_pos.x: %f, temp_pos.y: %f\n", temp_pos.x, temp_pos.y);
    }
}