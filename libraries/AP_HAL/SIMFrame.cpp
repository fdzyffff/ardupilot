#include "SIMState.h"

#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL

#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Helicopter.h>
#include <SITL/SIM_SingleCopter.h>
#include <SITL/SIM_Plane.h>
#include <SITL/SIM_QuadPlane.h>
#include <SITL/SIM_Rover.h>
#include <SITL/SIM_BalanceBot.h>
#include <SITL/SIM_Sailboat.h>
#include <SITL/SIM_MotorBoat.h>
#include <SITL/SIM_Tracker.h>
#include <SITL/SIM_Submarine.h>
#include <SITL/SIM_Blimp.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

using namespace AP_HAL;

void SIMState::_build_copter_frame() 
{
    switch (_sitl->sim_frame_type.get()) {
        default:
        case 0:
            sitl_model = SITL::MultiCopter::create("+");
            break;
        case 1:
            sitl_model = SITL::MultiCopter::create("quad");
            break;
        case 2:
            sitl_model = SITL::MultiCopter::create("copter");
            break;
        case 3:
            sitl_model = SITL::MultiCopter::create("x");
            break;
        case 4:
            sitl_model = SITL::MultiCopter::create("bfxrev");
            break;
        case 5:
            sitl_model = SITL::MultiCopter::create("bfx");
            break;
        case 6:
            sitl_model = SITL::MultiCopter::create("djix");
            break;
        case 7:
            sitl_model = SITL::MultiCopter::create("cwx");
            break;
        case 8:
            sitl_model = SITL::MultiCopter::create("hexa");
            break;
        case 9:
            sitl_model = SITL::MultiCopter::create("hexa-cwx");
            break;
        case 10:
            sitl_model = SITL::MultiCopter::create("hexa-dji");
            break;
        case 11:
            sitl_model = SITL::MultiCopter::create("octa");
            break;
        case 12:
            sitl_model = SITL::MultiCopter::create("octa-cwx");
            break;
        case 13:
            sitl_model = SITL::MultiCopter::create("octa-dji");
            break;
        case 14:
            sitl_model = SITL::MultiCopter::create("octa-quad-cwx");
            break;
        case 15:
            sitl_model = SITL::MultiCopter::create("dodeca-hexa");
            break;
        case 16:
            sitl_model = SITL::MultiCopter::create("tri");
            break;
        case 17:
            sitl_model = SITL::MultiCopter::create("y6");
            break;
        case 18:
            sitl_model = SITL::MultiCopter::create("deca");
            break;
        case 19:
            sitl_model = SITL::MultiCopter::create("deca-cwx");
            break;
    }
}

void SIMState::_build_heli_frame() 
{
    switch (_sitl->sim_frame_type.get()) {
        default:
        case 0:
            sitl_model = SITL::Helicopter::create("heli");
            break;
        case 1:
            sitl_model = SITL::Helicopter::create("heli-dual");
            break;
        case 2:
            sitl_model = SITL::Helicopter::create("heli-compound");
            break;
        case 3:
            sitl_model = SITL::Helicopter::create("heli-blade360");
            break;
    }
}

void SIMState::_build_plane_frame() 
{
    switch (_sitl->sim_frame_type.get()) {
        default:
        case 0:
            sitl_model = SITL::Plane::create("plane");
            break;
        case 1:
            sitl_model = SITL::Plane::create("plane-catapult");
            break;
        case 2:
            sitl_model = SITL::Plane::create("plane-bungee");
            break;
        case 3:
            sitl_model = SITL::Plane::create("plane-throw");
            break;
        case 4:
            sitl_model = SITL::Plane::create("plane-drop");
            break;
        case 10:
            sitl_model = SITL::QuadPlane::create("quadplane");
            break;
        case 12:
            sitl_model = SITL::QuadPlane::create("quadplane-octa-quad");
            break;
        case 13:
            sitl_model = SITL::QuadPlane::create("quadplane-octa");
            break;
        case 14:
            sitl_model = SITL::QuadPlane::create("quadplane-hexax");
            break;
        case 15:
            sitl_model = SITL::QuadPlane::create("quadplane-hexa");
            break;
        case 16:
            sitl_model = SITL::QuadPlane::create("quadplane-plus");
            break;
        case 17:
            sitl_model = SITL::QuadPlane::create("quadplane-y6");
            break;
        case 18:
            sitl_model = SITL::QuadPlane::create("quadplane-tri");
            break;
        case 19:
            sitl_model = SITL::QuadPlane::create("quadplane-tilttrivec");
            break;
        case 20:
            sitl_model = SITL::QuadPlane::create("quadplane-tilthvec");
            break;
        case 21:
            sitl_model = SITL::QuadPlane::create("quadplane-tilttri");
            break;
        case 22:
            sitl_model = SITL::QuadPlane::create("quadplanefirefly");
            break;
        case 23:
            sitl_model = SITL::QuadPlane::create("quadplanecl84");
            break;
        case 24:
            sitl_model = SITL::QuadPlane::create("quadplane-copter_tailsitter");
            break;
    }
}
#endif  // AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL