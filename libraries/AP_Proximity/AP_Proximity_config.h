#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PROXIMITY_ENABLED
#define HAL_PROXIMITY_ENABLED (!HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024)
#endif
