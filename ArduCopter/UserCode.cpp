#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    init_find_obj();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    update_find_obj();
    run_obj_wp();
    run_teststar();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::init_find_obj() {
    find_obj.last_ms = millis();
    find_obj.find_obj = false;
}

void Copter::update_find_obj() {
    uint32_t tnow = millis();
    if (tnow - find_obj.last_ms > 2000) {
        find_obj.find_obj = false;
    }
}

void Copter::set_find_obj() {
    find_obj.last_ms = millis();
    if (find_obj.find_obj == false) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"find obj!");
    }
    find_obj.find_obj = true;
}


// update mission
void Copter::run_teststar() {
    if (control_mode != TESTSTAR) {return;}
    AP_Mission::Mission_Command cmd;
    Vector3f tmp_neu;
    Location_Class tmp_loc;
    if (mode_teststar.TestStarState.do_next) {
        switch (mode_teststar.TestStarState.id) {
            case 0:
                {
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = MAX(mode_teststar.TestStarState.start_pos.z, 500.f); //cm
                    cmd.content.location.options = 0;
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 1:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(1.0f, 0, 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 2:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 3:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 4:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 5:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 6:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos + Vector3f(1.0f, 0, 0) * mode_teststar.TestStarState.radius_cm;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    //如果经纬度设0，则原地降落，如果不为0，则按照设置的高度飞过去再降落
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0;
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            case 7:
                {
                    tmp_neu = mode_teststar.TestStarState.start_pos;
                    tmp_loc = Location_Class(tmp_neu);
                    cmd.id = MAV_CMD_NAV_LAND;
                    //如果经纬度设0，则原地降落，如果不为0，则按照设置的高度飞过去再降落
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt*2; //int32_t cm
                    cmd.content.location.options = 0;
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    mode_teststar.start_command(cmd);
                    mode_teststar.TestStarState.do_next = false;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"CMD: %d ", mode_teststar.TestStarState.id);
                    break;
                }
            default:
                break;
        }
    }
    if ((mode_teststar.TestStarState.state == ModeTestStar::TestStarSub_Flying) && mode_teststar.verify_command_callback(mode_teststar.TestStarState.current_cmd)) {
        if (mode_teststar.TestStarState.id < 7) {
            mode_teststar.TestStarState.do_next = true;
            mode_teststar.TestStarState.id += 1;
        }
    }
}

void Copter::run_obj_wp() {
    if (control_mode != TESTSTAR) {return;}
    if (mode_teststar.mode() != Auto_WP) {
        //mode_teststar.init_TestStart();
        return;}

    AP_Mission::Mission_Command cmd;
    Location_Class tmp_loc;
    switch (mode_teststar.TestStarState.state) {
        case ModeTestStar::TestStarSub_Flying: {
            if (find_obj.find_obj) {
                mode_teststar.TestStarState.dest = wp_nav->get_wp_destination();
                mode_teststar.TestStarState.origin = wp_nav->get_wp_origin();
                mode_teststar.TestStarState.dir = wp_nav->get_yaw();
                Vector3f stopping_point;
                wp_nav->get_wp_stopping_point(stopping_point);

                tmp_loc = Location_Class(stopping_point);
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                cmd.content.location.options = 0; //alt from sea level
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务

                mode_teststar.start_command(cmd);
                //mode_teststar.wp_start(stopping_point);
                mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f, 0.0f, 0, false);
                mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_brake;
            }
        }
        break;
        case ModeTestStar::TestStarSub_brake: {
            if (mode_teststar.verify_command_callback(mode_teststar.TestStarState.current_cmd)) {
                tmp_loc = Location_Class(calculate_step1());
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                cmd.content.location.options = 0; //alt from sea level
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                mode_teststar.start_command(cmd);

                //mode_teststar.wp_start(calculate_step1());
                mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f, 0.0f, 0, false);
                mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_step1;
            }
        }
        break;
        case  ModeTestStar::TestStarSub_step1:  {
            if (mode_teststar.verify_command_callback(mode_teststar.TestStarState.current_cmd)) {
                if (find_obj.find_obj) {
                    tmp_loc = Location_Class(calculate_step1());
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);

                    //mode_teststar.wp_start(calculate_step1());
                    mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f, 0.0f, 0, false);
                    mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_step1;
                }
                else {
                    tmp_loc = Location_Class(calculate_step2());
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);

                    //mode_teststar.wp_start(calculate_step2());
                    mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f - 90.f, 0.0f, 0, false);
                    mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_step2;
                }
            }
        }
        break;
        case  ModeTestStar::TestStarSub_step2:  {
            if (mode_teststar.verify_command_callback(mode_teststar.TestStarState.current_cmd)) {
                if (find_obj.find_obj) {
                    tmp_loc = Location_Class(calculate_step2());
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);

                    //mode_teststar.wp_start(calculate_step2());
                    mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f - 90.f, 0.0f, 0, false);
                } else {
                    tmp_loc = Location_Class(calculate_step3());
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                    cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                    cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                    cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                    cmd.content.location.options = 0; //alt from sea level
                    //if want to set alt from home point add following line
                    cmd.content.location.flags.relative_alt = 1; //alt from home point
                    cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                    mode_teststar.start_command(cmd);
                    //mode_teststar.wp_start(calculate_step3());
                    mode_teststar.auto_yaw.set_mode_to_default(false);
                    mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_step3;
                }
            }
        }
        break;
        case  ModeTestStar::TestStarSub_step3:  {
            if (mode_teststar.verify_command_callback(mode_teststar.TestStarState.current_cmd)) {
                tmp_loc = Location_Class(mode_teststar.TestStarState.dest);
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                cmd.content.location.options = 0; //alt from sea level
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                mode_teststar.start_command(cmd);

                //mode_teststar.wp_start(mode_teststar.TestStarState.dest);
                mode_teststar.auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_Flying;
            } else if (find_obj.find_obj) {
                tmp_loc = Location_Class(calculate_step2());
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.content.location.lat = tmp_loc.lat; //int32_t 10e7
                cmd.content.location.lng = tmp_loc.lng; //int32_t 10e7
                cmd.content.location.alt = tmp_loc.alt; //int32_t cm
                cmd.content.location.options = 0; //alt from sea level
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                mode_teststar.start_command(cmd);
                //mode_teststar.wp_start(calculate_step2());
                mode_teststar.auto_yaw.set_fixed_yaw(mode_teststar.TestStarState.dir*0.01f - 90.f, 0.0f, 0, false);
                mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_step2;
            }
        }
        break;
        default:
            break;
    }
}

void Copter::init_TestStart()
{
    mode_teststar.TestStarState.origin = Vector3f(0.0f, 0.0f, 0.0f);
    mode_teststar.TestStarState.dest = Vector3f(0.0f, 0.0f, 0.0f);
    mode_teststar.TestStarState.start_pos = inertial_nav.get_position();
    mode_teststar.TestStarState.dir = 0.0f;
    mode_teststar.TestStarState.state = ModeTestStar::TestStarSub_Flying;
    mode_teststar.TestStarState.do_next = true;
    mode_teststar.TestStarState.id = 0;
    mode_teststar.TestStarState.radius_cm = 5000.0;
}

Vector3f Copter::calculate_step1() {
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(mode_teststar.TestStarState.dir*0.01f));
    Vector3f tmp_pos = copter.inertial_nav.get_position() + (tmp_m * Vector3f(0.0f, 200.f, 0.0f));
    return tmp_pos;
}

Vector3f Copter::calculate_step2() {
    Matrix3f tmp_m;
    tmp_m.from_euler(0.0f, 0.0f, radians(mode_teststar.TestStarState.dir*0.01f));
    Vector3f tmp_pos = copter.inertial_nav.get_position() + (tmp_m * Vector3f(200.0f, 0.0f, 0.0f));
    return tmp_pos;
}

Vector3f Copter::calculate_step3() {
    Vector2f origin_xy(mode_teststar.TestStarState.origin.x, mode_teststar.TestStarState.origin.y);
    Vector2f dest_xy(mode_teststar.TestStarState.dest.x, mode_teststar.TestStarState.dest.y);
    Vector2f curr_xy(copter.inertial_nav.get_position().x, copter.inertial_nav.get_position().y);
    Vector2f dir = (dest_xy - origin_xy);
    if (dir.length() < 200.f)
    {
        return mode_teststar.TestStarState.dest;
    }
    dir.normalize();
    float dir_length = dir * (curr_xy - origin_xy);
    Vector3f tmp_pos = mode_teststar.TestStarState.origin + (Vector3f(dir.x, dir.y, 0.0f) * dir_length);
    return tmp_pos;
}
