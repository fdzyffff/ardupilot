// #include "Copter.h"

// #if MODE_GUIDED_ENABLED

// // init - initialise guided controller
// bool ModeMission::init(bool ignore_checks)
// {
//     return copter.mode_guided.init();
// }

// // run - runs the guided controller
// // should be called at 100hz or more
// void ModeMission::run()
// {
//     update_state();

//     switch (mission_state) {
//         case State::Init:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Takeoff:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Wait:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Cruise:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Search:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Track:
//         {
//             copter.mode_guided.run();
//         }
//         break;
//         case State::Return:
//         {
//             copter.mode_rtl.run();
//         }
//         break;
//     }
// }

// void ModeMission::update_state()
// {

//     switch (mission_state) {
//         case State::Init:
//         {
//             if (mode_guided.guided_mode != SubMode::VelAccel) {
//                 copter.mode_guided.pva_control_start();
//             }
//         }
//         break;
//         case State::Takeoff:
//         {
//             if (mode_guided.guided_mode != SubMode::TakeOff) {
//                 ;
//             }
//         }
//         break;
//         case State::Wait:
//         {
//             if (mode_guided.guided_mode != SubMode::VelAccel) {
//                 copter.mode_guided.pva_control_start();
//             }
//             if (copter.umission.valid()) {
//                 set_state(State::Cruise);
//             }
//         }
//         break;
//         case State::Cruise:
//         {
//             if (mode_guided.guided_mode != SubMode::PosVelAccel) {
//                 copter.mode_guided.posvelaccel_control_start();
//             }
//             if (millis() - copter.mode_guided.update_time_ms > 1000) {
//                 copter.mode_guided.set_destination_posvel(xxx);
//             }
//             if (dist < 200.f) {
//                 set_state(State::Search);
//             }
//         }
//         break;
//         case State::Search:
//         {
//             if (!copter.umission.valid()) {
//                 set_state(State::Wait);
//             }
//             if (mode_guided.guided_mode != SubMode::PosVelAccel) {
//                 copter.mode_guided.posvelaccel_control_start();
//             }
//             if (millis() - copter.mode_guided.update_time_ms > 1000) {
//                 copter.mode_guided.set_destination_posvel(xxx);
//             }
//             if (find_target && dist < 200.f && have_target) {
//                 set_state(State::Track);
//             }
//         }
//         break;
//         case State::Track:
//         {
//             if (mode_guided.guided_mode != SubMode::PosVelAccel) {
//                 copter.mode_guided.posvelaccel_control_start();
//             }
//             if (millis() - copter.mode_guided.update_time_ms > 1000) {
//                 copter.mode_guided.set_destination_posvel(xxx);
//             }
//             if (!find_target) {
//                 set_state(State::Search);
//             }
//         }
//         break;
//         case State::Return:
//         {
//             ;
//         }
//         break;
//     }
// }

// void ModeMission::set_state(State::state_in)
// {
//     if (mission_state == state_in) {
//         return;
//     }

//     switch (mission_state) {
//         case State::Init:
//         {
//             mission_state = state_in;
//         }
//         break;
//         case State::Takeoff:
//         {
//             if (mode_guided.do_user_takeoff_start(200.f)) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Takeoff");
//             }
//         }
//         break;
//         case State::Wait:
//         {
//             if (copter.mode_guided.pva_control_start()) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Wait");
//             }
//         }
//         break;
//         case State::Cruise:
//         {
//             if (copter.mode_guided.pva_control_start()) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Cruise");
//             }
//         }
//         break;
//         case State::Search:
//         {
//             if (copter.mode_guided.pva_control_start()) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Search");
//             }
//         }
//         break;
//         case State::Track:
//         {
//             if (copter.mode_guided.pva_control_start()) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Track");
//             }
//         }
//         break;
//         case State::Return:
//         {
//             if (copter.mode_rtl.init()) {
//                 mission_state = state_in;
//             } else {
//                 gcs().send_text(MAV_SEVERITY_INFO, "[Mis] State: Return");
//             }
//         }
//         break;
//     }
// }
