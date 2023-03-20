#include "mode.h"
#include "Plane.h"
/*
  mode sub_althold parameters
 */
const AP_Param::GroupInfo ModeSubAltHold::var_info[] = {

    AP_SUBGROUPINFO(_vel_pid, "VEL_",  1, ModeSubAltHold, AC_PID),

    AP_GROUPINFO("VEL_MAX", 2, ModeSubAltHold, _max_vel_cms, 200.f),

    AP_GROUPINFO("ACC_MAX", 3, ModeSubAltHold, _max_acc_cmss, 50.f),

    AP_GROUPINFO("POS_P", 4, ModeSubAltHold, _pos_p, 1.0f),
//     // @Param: LVL_ALT
//     // @DisplayName: Takeoff mode altitude level altitude
//     // @Description: This is the altitude below which wings are held level for TAKEOFF mode
//     // @Range: 0 50
//     // @Increment: 1
//     // @Units: m
//     // @User: Standard
//     AP_GROUPINFO("LVL_ALT", 2, ModeTakeoff, level_alt, 20),

//     // @Param: LVL_PITCH
//     // @DisplayName: Takeoff mode altitude initial pitch
//     // @Description: This is the target pitch for the initial climb to TKOFF_LVL_ALT
//     // @Range: 0 30
//     // @Increment: 1
//     // @Units: deg
//     // @User: Standard
//     AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, level_pitch, 15),

//     // @Param: DIST
//     // @DisplayName: Takeoff mode distance
//     // @Description: This is the distance from the takeoff location where the plane will loiter. The loiter point will be in the direction of takeoff (the direction the plane is facing when the motor starts)
//     // @Range: 0 500
//     // @Increment: 1
//     // @Units: m
//     // @User: Standard
//     AP_GROUPINFO("DIST", 4, ModeTakeoff, target_dist, 200),
    

    AP_GROUPEND

};

bool ModeSubAltHold::_enter()
{
    // if we haven't run the rate controllers for 2 seconds then
    // reset the integrators
    plane.rollController.reset_I();
    plane.pitchController.reset_I();
    plane.yawController.reset_I();
    plane.as_rollController.reset_I();
    plane.as_pitchController.reset_I();
    plane.as_yawController.reset_I();

    // and reset steering controls
    plane.steer_state.locked_course = false;
    plane.steer_state.locked_course_err = 0;

    _target_alt_cms = plane.depth_sensor.get_altitude()*100.f;

    gcs().send_text(MAV_SEVERITY_INFO, "In Sub-AltHold Mode");
    return true;
}

void ModeSubAltHold::update()
{
    if (_max_vel_cms < 10.f) {_max_vel_cms.set(10.f);}
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;

    float pitch_input = plane.channel_pitch->norm_input();
    get_target_alt_cm(pitch_input);
    float target_vel = get_target_vel_cms(_target_alt_cms);
    plane.nav_pitch_cd = get_target_pitch_cd(target_vel);

    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());

    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    subalt_stat.target_input = plane.channel_pitch->norm_input();
    subalt_stat.target_climb_rate = target_vel;
    subalt_stat.target_depth = _target_alt_cms;
}

void ModeSubAltHold::get_target_alt_cm(float pitch_input)
{
    float curr_alt_cms = plane.depth_sensor.get_altitude()*100.f;
    float target_delta_v = pitch_input*_max_vel_cms;
    _target_alt_cms = constrain_float(_target_alt_cms+target_delta_v*plane.G_Dt, curr_alt_cms - _max_vel_cms*1.5f, curr_alt_cms + _max_vel_cms*1.5f);
}

float ModeSubAltHold::get_target_vel_cms(float target_alt_cms)
{
    float curr_alt_cms = plane.depth_sensor.get_altitude()*100.f;
    float error_alt_cms = target_alt_cms - curr_alt_cms;
    float targt_vel_cms = sqrt_controller(error_alt_cms, _pos_p, _max_acc_cmss, plane.G_Dt);
    return targt_vel_cms;
}

int32_t ModeSubAltHold::get_target_pitch_cd(float target_vel_cms)
{
    float curr_vel_cms = plane.depth_sensor.get_climb_rate()*100.f;
    float target_pitch_norm = constrain_float(_vel_pid.update_all(target_vel_cms, curr_vel_cms)/_max_vel_cms, -1.0f, 1.0f);
    float scalar = 1.0f;
    target_pitch_norm = constrain_float(target_pitch_norm*scalar, -1.0f, 1.0f);
    int32_t target_pitch_cd = 0;
    if (target_pitch_norm > 0) {
        target_pitch_cd = target_pitch_norm * plane.aparm.pitch_limit_max_cd;
    } else {
        target_pitch_cd = -(target_pitch_norm * plane.pitch_limit_min_cd);
    }
    return target_pitch_cd;
}

void ModeSubAltHold::_exit()
{
    plane.as_rollController.reset_I();
    plane.as_pitchController.reset_I();
    plane.as_yawController.reset_I();
}
