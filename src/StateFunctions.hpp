#include <stdint.h>
#include "VL53L0X.h"
#include "Ultrasonic.hpp"
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
// PID parameters for orientation control.
#define ORIENT_P 10.0f
#define ORIENT_I 0.0f
#define ORIENT_D 0.0f
// 
// PID parameters for location control.
#define LOCATE_P 0.05f
#define LOCATE_I 0.0f
#define LOCATE_D 0.0f
// 
// The set distance desired to the wall.
#define WALL_JUMP_DIST_INIT_VEL 500.0f
// 
// The set distance with no initial velocity.
#define WALL_JUMP_DIST_0_VEL 300.0f
//
// Time in ms to get off the platform
#define DRIVE_OFF_PLATFORM_TIME 2000
// 
// The amount in G that is considered moving upwards in z.
#define JUMP_THRESHOLD 0.5f
// 
// The tolerance for yaw re-orientation.
#define YAW_TOL 3.0f
// 
// The tolerance for height detection.
#define HEI_TOL 0.06f
//
// The tolerance of change in distance to count as finding the pole.
#define POLE_DELTA_TOLERANCE 15.0f
// 
// The ceiling of the noise.
#define POLE_NOISE_CEIL 200.0f
//
// The tolerance of change in distance between the measured data and subsequent measurements that counts as the same.
#define POLE_CONFIRMATION_TOLERANCE 15.0
//
// The number of times to verify the pole search measurements before confirming there is indeed a pole there.
#define POLE_CONFIRMATION_CHECK_COUNT 3
//
// The number of times the confirmation check can fail (false negative).
#define POLE_CONFIRMATION_CHECK_FAIL_TOLERANCE 1
//
// The angle for the robot to turn each time it's orienting
#define WALL_ORIENT_ANGLE 5.0f
//
// The number of times the sensor should be read for median filter
#define WALL_ORIENT_READ_COUNT 3
//
// The tolerance of change in distance to count as oriented
#define WALL_ORIENT_TOLERANCE 3.0f
//
// The tolerance of change between sum of ultrasonics and width of field to count as oriented. Should include distance between ultrasonics
// Distance between ultrasonics = 20
#define WALL_ORIENT_ULTRASONIC_TOLERANCE 25.0f
// 
// The tolerance of change between measured data and subsequent measurements that counts as ramp
#define RAMP_CONFIRMATION_TOLERANCE 15.0f
// 
// The outer limit tolerance of change between measured data and subsequent measurements that counts as pole
#define RAMP_CONFIRMATION_TOLERANCE 30.0f
//
// Time for the IMU to stablize, in milliseconds.
#define IMU_STABLIZE_TIME 2000
// 
// Drive sleep time.
#define DRIVE_SLEEP_TIME 30
// 
// Functions in the state machine.
namespace StateFunctions 
{
    //
    // State that waits for the start button to be pressed.
    int waitForStartButton(Adafruit_DCMotor *jumpMotor);
    //
    // State that samples the IMU for reference yaw.
    int sampleYaw(IMU *imu, float &yaw);
    // 
    // Just drives straight.
    int driveStraight(Drive *drive, float speed, uint32_t timeMs);
    // 
    // Help create the approach curve.
    int curveHelper(Drive *drive, uint32_t time);
    // 
    // Approach the wall by a set distance.
    int approachAndStop(Drive *drive, VL53L0X* prox);
    // 
    // Another state for approaching.
    int approach2(Drive* drive, VL53L0X* prox);
    // 
    // State that manages the jumping.
    int jump(Adafruit_DCMotor *jumpMotor, Drive* drive);
    // 
    // State that manages the in air control.
    int inAir(Drive *drive, IMU *imu);
    // 
    // Check if the robot is upside down and set controls as needed.
    int checkUpsideDown(Drive* drive, VL53L0X* prox);
    // 
    // Orient the robot to be parallelish to the wall.
    int orient(Drive *drive, VL53L0X *prox, Ultrasonic* ultrasonicL, Ultrasonic* ultrasonicR);
    // 
    // Function that helps orient the robot after jumping.
    int orientForwardIMU(Drive *drive, IMU *imu, float refYaw);
    // 
    // State that searches for the destination.
    int locateDest(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight, VL53L0X *prox);
    //
    // State that drives to destination
    int driveToDest(Drive *drive, IMU *imu);
    //
    // State that searches for destination using general algorithm
    int poleSearchGeneral(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight, VL53L0X *prox);
}

#endif /* __STATE_FUNCTIONS__ */
