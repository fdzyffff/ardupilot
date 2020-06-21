#include "Plane.h"

void Plane::HB1_status_init() {
    HB1_status_set_HB_Power_Action(HB1_PoserAction_None);
}

void Plane::HB1_status_update_20Hz() {
    HB1_Power_update();
}

void Plane::HB1_Power_update() {
    ;
}

void Plane::HB1_status_set_HB_Power_Action(HB1_Power_Action_t action) {
    HB_Power_Action = action;
    switch (HB_Power_Action) {
        case HB1_PoserAction_None:
            gcs().send_text(MAV_SEVERITY_INFO, "Power None");
            break;
        case HB1_PoserAction_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket ON");
            break;
        case HB1_PoserAction_EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine ON");
            break;
        case HB1_PoserAction_EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine OFF");
            break;
        case HB1_PoserAction_ParachuteON:
            gcs().send_text(MAV_SEVERITY_INFO, "Parachute ON");
            break;
        default:
            break;
    }
}