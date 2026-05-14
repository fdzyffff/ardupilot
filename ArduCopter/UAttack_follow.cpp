#include "Copter.h"

void UAttack::update_target_loc()
{
    float max_dist_m = 1000.f;
    int32_t alt_cm = 0;
    if (!copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm)) {
        return;
    }
    if (!is_active()) {
        return;
    }
    float alt_relative_m = (float)alt_cm * 0.01f;
    float angle_max = degrees(atanf(-alt_relative_m/max_dist_m));
    if (ef_info.y > angle_max) {
        // printf("efinfo.y: %f, angle_max: %f\n", ef_info.y, angle_max);
        return;
    }
    if (ef_info.y > -10.0f) {
        // gcs().send_text(MAV_SEVERITY_INFO, "Tgt cal ERR");
        return;
    }
    float dist_m = alt_relative_m / tanf(-radians(ef_info.y));
    float bearing_deg = ef_info.x;
    Location tmp_loc = copter.current_loc;
    // printf("dist_m: %f\n", dist_m);
    tmp_loc.offset_bearing(bearing_deg, dist_m);
    tmp_loc.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
    set_target_loc(tmp_loc);
}

void UAttack::set_target_loc(Location& loc_in)
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

bool UAttack::have_target_loc()
{
    if (millis() - _last_target_update_ms < 5000) {
        return true;
    }
    return false;
}
