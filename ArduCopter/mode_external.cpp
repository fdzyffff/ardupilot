#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeExternal::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        set_stage(stage_class::Wait);
        gcs().send_text(MAV_SEVERITY_INFO, "In Ext MODE");
        return true;
    }
    return false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeExternal::run()
{
    update_stage();

    switch (stage) {
        case stage_class::Wait:
        {
            copter.mode_guided.run();
        }
        break;
        case stage_class::ANGLE:
        {
            update_angle();
            copter.mode_guided.run();
        }
        break;
        case stage_class::VEL:
        {
            update_vel();
            copter.mode_guided.run();
        }
        break;
        case stage_class::WP:
        {
            update_wp();
            copter.mode_guided.run();
        }
        break;
        case stage_class::ATK:
        {
            update_attack();
            copter.mode_guided.run();
        }
        break;
        case stage_class::HOVER:
        {
            update_hover();
            copter.mode_circle.run();
        }
        break;
    }
}

void ModeExternal::update_stage()
{
    if (copter.uart.control_status.valid) {
        switch (copter.uart.control_status.type) {
            case 0x1A:
            {
                set_stage(stage_class::ANGLE);
                break;
            }
            case 0x3C:
            {
                set_stage(stage_class::VEL);
                break;
            }
            case 0x55:
            {
                set_stage(stage_class::WP);
                break;
            }
            case 0xFE:
            {
                set_stage(stage_class::ATK);
                break;
            }
            default:
            {
                set_stage(stage_class::Wait);
                break;
            }
        }
    } else {
        set_stage(stage_class::HOVER);
    }
}

void ModeExternal::update_angle()
{
    // float target_roll = copter.uart.control_status.cmd_roll;
    // float target_pitch = copter.uart.control_status.cmd_pitch;
}

void ModeExternal::update_vel()
{

}

void ModeExternal::update_attack()
{
    Vector3f vel;
    vel.x = copter.uart.control_status.cmd_vel_x * 100.f;
    vel.y = copter.uart.control_status.cmd_vel_y * 100.f;
    vel.z = copter.uart.control_status.cmd_vel_z * 100.f;

    bool use_yaw = false;
    float yaw_cd = 0.0f;
    bool use_yaw_rate = true;
    float yaw_rate_cds = 0.0f;
    bool yaw_relative = false;
    bool log_request = false;

    if (vel.xy().length_squared() > (100.0 * 100.0)) {
        yaw_cd = get_bearing_cd(Vector2f{}, vel.xy());
        use_yaw = true;
        use_yaw_rate = false;
        yaw_rate_cds = 0.0f;
    }

    copter.mode_guided.set_velocity(vel, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative, log_request);
}

void ModeExternal::update_wp()
{
    if (millis() - _last_loc_update_time_ms > 1000) {
        _last_loc_update_time_ms = millis();
        Location tmp_loc = copter.uart.control_status.cmd_loc;
        copter.mode_guided.set_destination(tmp_loc);
    }
}

void ModeExternal::update_hover()
{
    ;
}

void ModeExternal::set_stage(stage_class stage_in)
{
    if (stage == stage_in) {
        return;
    }

    switch (stage_in) {
        case stage_class::Wait:
        {
            copter.mode_guided.velaccel_control_start();
            stage = stage_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Wait");
        }
        break;
        case stage_class::ANGLE:
        {
            copter.mode_guided.velaccel_control_start();
            stage = stage_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: ANGLE");
        }
        break;
        case stage_class::VEL:
        {
            copter.mode_guided.velaccel_control_start();
            stage = stage_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: VEL");
        }
        break;
        case stage_class::WP:
        {
            copter.mode_guided.velaccel_control_start();
            stage = stage_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: WP");
        }
        break;
        case stage_class::HOVER:
        {
            if (copter.mode_circle.init(false)) {
                stage = stage_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: HOVER");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: HOVER");
            }
        }
        break;
        case stage_class::ATK:
        {
            copter.mode_guided.velaccel_control_start();
            stage = stage_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: ATK");
        }
        break;
        default:
        break;
    }
}

bool ModeExternal::is_taking_off() const
{
    return false;
}

uint32_t ModeExternal::wp_distance() const
{
    switch(stage) {
        case stage_class::Wait:
        case stage_class::WP:
        case stage_class::ATK:
            return copter.mode_guided.wp_distance();
            break;
        case stage_class::HOVER:
            return copter.mode_circle.wp_distance();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

int32_t ModeExternal::wp_bearing() const
{
    switch(stage) {
        case stage_class::Wait:
        case stage_class::VEL:
        case stage_class::ANGLE:
        case stage_class::ATK:
        case stage_class::WP:
            return copter.mode_guided.wp_bearing();
            break;
        case stage_class::HOVER:
            return copter.mode_circle.wp_bearing();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

float ModeExternal::crosstrack_error() const
{
    switch(stage) {
        case stage_class::Wait:
        case stage_class::VEL:
        case stage_class::ATK:
        case stage_class::WP:
            return copter.mode_guided.crosstrack_error();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}
#endif