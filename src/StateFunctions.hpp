#include <stdint.h>
#include "VL53L0X.h"
#include "Drive.hpp"
#include "PID.hpp"
#include "PinDefines.h"
#include "IMU_Wrapper.hpp"

#ifndef __STATE_FUNCTIONS__
#define __STATE_FUNCTIONS__

// 
// PID parameters for the ACC.
#define ACC_P 0.1f
#define ACC_I 0.0f
#define ACC_D 0.0f
// 
// PID parameters for in air control.
#define AIR_P 1.0f
#define AIR_I 1.0f
#define AIR_D 1.0f
// 
// The set distance desired to the wall.
#define WALL_SET_DIST 100.0f
#define WALL_DIST_TOL 10.0f
//
// Time in ms to get off the platform
#define DRIVE_OFF_PLATFORM_TIME 500
// 
// The amount in G that is considered moving upwards in z.
#define JUMP_THRESHOLD 0.5f

namespace StateFunctions 
{
    //
    // State that waits for the start button to be pressed.
    int waitForStartButton(IMU *imu, float &yaw);
    //
    // State to get off the starting platform.
    int getOffPlateform(Drive* drive);
    //
    // State that approaches the wall of the arena.
    int approach(Drive* drive, VL53L0X* prox);
    // 
    // State that manages the jumping.
    int jump(Adafruit_DCMotor *jumpMotor, IMU *imu);
    // 
    // State that manages the in air control.
    int inAir(Drive *drive, IMU *imu);
    // 
    // Function that helps orient the robot after jumping.
    int orientForward(Drive *drive, IMU *imu, float yaw);
    // 
    // State that searches for the destination.
    int locateDest(Drive *drive /* Other components TBD. */);

}

#endif /* __STATE_FUNCTIONS__ */
