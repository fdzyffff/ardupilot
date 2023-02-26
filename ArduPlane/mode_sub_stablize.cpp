#include "mode.h"
#include "Plane.h"
/*
  mode sub_stablize parameters
 */
// const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
//     // @Param: ALT
//     // @DisplayName: Takeoff mode altitude
//     // @Description: This is the target altitude for TAKEOFF mode
//     // @Range: 0 200
//     // @Increment: 1
//     // @Units: m
//     // @User: Standard
//     AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, 50),

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
    
//     AP_GROUPEND
// };

bool ModeSubStablize::_enter()
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

    return true;
}

void ModeSubStablize::run()
{

}

void ModeSubStablize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

void ModeSubStablize::_exit()
{

}
