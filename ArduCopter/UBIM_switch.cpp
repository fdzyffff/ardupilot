#include "Copter.h"

bool UBIM::switch_back_to_wp()
{
    if (!uav_unlock) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV hover reject, set unlock first!");
        return false;
    }

    if (!copter.motors->armed()) // || copter.ap.land_complete
    {
        if (!copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND))
        {
            gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV can NOT start");
            return false;
        }
        if (!copter.arming.arm(AP_Arming::Method::MAVLINK))
        {
            gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV can NOT start");
            return false;
        }
    }

    if (!copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV can NOT AUTO");
        return false;
    }

    return true;
}

bool UBIM::switch_hover()
{
    if (!uav_unlock) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV hover reject, set unlock first!");
        return false;
    }

    if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV can NOT start");
        return false;
    }

    copter.mode_guided.init(false);

    return true;
}

bool UBIM::switch_unlock()
{
    if (!uav_unlock) {
        uav_unlock = true;
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV unlock");
        copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND);
        copter.mode_auto.mission.clear();
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV wp clear %d", copter.mode_auto.mission.num_commands());
    } else {
        // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV already unlock");
    }
    return true;
}

bool UBIM::switch_manual()
{
    if (!uav_unlock) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV hover reject, set unlock first!");
        return false;
    }
    if (!uav_manual) {
        uav_manual = true;
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV manual");
    } else {
        // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV already manual");
    }
    return true;
}

bool UBIM::switch_land()
{
    if (!uav_unlock) {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV hover reject, set unlock first!");
        return false;
    }
    if (!uav_manual) 
    {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV land reject, set manual first!");
        return false;
    }
    if (copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) 
    {
        // gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV land");
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "BIM: UAV land fail");
        return false;
    }
    return true;
}
