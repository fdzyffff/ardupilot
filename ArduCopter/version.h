#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "[AC432]SKB-v1.0.5"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,3,2,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 3
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#include <AP_Common/AP_FWVersionDefine.h>

// Log
// ArduCopter V4.3.2-SKB-v1.0.2
// Add engine msg forward
// ArduCopter V4.3.2-SKB-v1.0.1
// refine the engine brake procedure