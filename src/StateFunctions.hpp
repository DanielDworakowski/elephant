#include <stdint.h>
#include "VL53L0X.h"
#include "Drive.hpp"
#include "PID.hpp"
#include "PinDefines.h"

#ifndef __STATE_FUNCTIONS__
#define __STATE_FUNCTIONS__

// 
// PID parameters for the ACC.
#define ACC_P 0.1f
#define ACC_I 0.0f
#define ACC_D 0.0f
#define SPEED_MAX 4.0f
#define SPEED_MIN -4.0f
// 
// The set distance desired to the wall.
#define WALL_SET_DIST 100.0f
#define WALL_DIST_TOL 10.0f
//
// Time in ms to get off the platform
#define DRIVE_OFF_PLATFORM_TIME 500

namespace StateFunctions 
{
    //
    // State that approaches the wall of the arena.
    int approachState(Drive* drive, VL53L0X* prox);
    //
    // State that waits for the start button to be pressed.
    int waitForStartButton();
    //
    // State to get off the starting platform.
    int getOffPlateform(Drive* drive);
}

#endif /* __STATE_FUNCTIONS__ */
