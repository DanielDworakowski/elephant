#include <stdint.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "PID.hpp"

#ifndef __DRIVE__
#define __DRIVE__
// 
// Left motor tuning.
#define LEFT_MOTOR_P 9.0f
#define LEFT_MOTOR_I 0.5f
#define LEFT_MOTOR_D 0.0f
// 
// Right motor tuning.
#define RIGHT_MOTOR_P 8.0f
#define RIGHT_MOTOR_I 0.5f
#define RIGHT_MOTOR_D 0.0f
// 
// Yaw control tuning.
#define YAW_P 7.0f 
#define YAW_I 1.0f
#define YAW_D 0.0f
// 
// Tuning for the second orientation of the robot.
// Left motor tuning.
#define POLE_LEFT_MOTOR_P 14.0f
#define POLE_LEFT_MOTOR_I 1.5f
#define POLE_LEFT_MOTOR_D 0.0f
// 
// Right motor tuning.
#define POLE_RIGHT_MOTOR_P 10.0f
#define POLE_RIGHT_MOTOR_I 1.5f // 0.0f
#define POLE_RIGHT_MOTOR_D 0.0f
// 
// Yaw control tuning.
#define POLE_YAW_P 20.0f 
#define POLE_YAW_I 5.0f
#define POLE_YAW_D 0.0f
// 
// Tuning for the second orientation of the robot.
// Left motor tuning.
#define TURN_LEFT_MOTOR_P 20.0f
#define TURN_LEFT_MOTOR_I 4.0f
#define TURN_LEFT_MOTOR_D 0.0f
// 
// Right motor tuning.
#define TURN_RIGHT_MOTOR_P 20.0f
#define TURN_RIGHT_MOTOR_I 4.0f
#define TURN_RIGHT_MOTOR_D 0.0f
// 
// Yaw control tuning.
// #define TURN_YAW_P 20.0f  // Radian tuning.
// #define TURN_YAW_I 0.0f
// #define TURN_YAW_D 0.03f
#define TURN_YAW_P 1.0f 
#define TURN_YAW_I 0.0f
#define TURN_YAW_D 0.0f
// 
// Tolerance for the rotation
// #define YAW_TOLERANCE (5.0f * M_PI / 180.0f) // 2 degree tolerance.
#define YAW_TOLERANCE 3.0f
// 
// The maximum and minimum motor speeds.
// Given we have a 11.1V LiPo divide 255 max by 2.
#define MAX_SPEED 255/*(255 / 2.0f)*/
#define MIN_SPEED -255/*(-255 / 2.0f)*/
// 
// The maximum and miminmum robot velocity.
#define ROBOT_SPEED_MAX 4.0f
#define ROBOT_SPEED_MIN -4.0f
// 
// The radius of the wheels.
#define WHEEL_RADIUS (0.1524f)
#define WHEEL_DIAMETER (2 * 0.1524f)
#define WHEEL_CIRCUMFRENCE (M_PI * WHEEL_DIAMETER)
#define CHASIS_LENGTH (0.28f) 
// 
// Conversion factors from ticks to linear velocity.
#define TICKS_PER_REVOLUTION (12.0f * 47.0f)
#define TO_OMEGA(ticks) ((ticks) / TICKS_PER_REVOLUTION * 2.0f * M_PI)
// 
// x_dot = r/2(v_r + v_l)cos(phi)
// y_dot = "            "sin(phi)
// phi_dot = R/L(v_r - v_l)
// 
// v_r = (2(v_desired) + omega_desiged*L) / 2r
// v_l = (2(v_desired) - omega_desiged*L) / 2r
class Drive {
    public:
        Drive(volatile int32_t *rEncoderCount, volatile int32_t *lEncoderCount, Adafruit_DCMotor *rMotor, Adafruit_DCMotor *lMotor);
        ~Drive();
        // 
        // Resets the time to now - tDiff, this prevents integration error.
        int reset(uint32_t tDiff);
        // 
        // Make the robot drive straight from the current trajectory.
        int goStraight();
        // 
        // Manage PID loops.
        int update();
        // 
        // Change the reference values desired.
        int setReference(float setSpeed, float setOmega);
        // 
        // Only set the current omega.
        int setOmega(float setOmega);
        // 
        // Stop the robot.
        int stop();
        // 
        // Reset the position vector's origin.
        int resetOrigin();
        //
        // Turn the robot by some angle. 
        int turnTheta(float theta);
        // 
        // Set the tuning of the controllers for pole searching.
        int setPoleSearch();
        // 
        // Pole search.
        int setJump();
        //
        // Turning. 
        int setTurn();
        // 
        // Set whether the robot is upside down to invert control. 
        int setUpsideDown(bool upsideDown);

    private:
        // 
        // Drive state.
        enum DriveState: byte
        {
            TURN_STATE,
            JUMP_STATE,
            POLE_SEARCH_STATE
        };
        // 
        // Calculate the compensation for angular speeds.
        int angleComp(int32_t lenc, int32_t renc, float dT, float &setOmega);
        // 
        // Calculate the speeds of the wheels to achieve set speeds.
        int calcWheelSpeeds(float setSpeed, float setOmega);
        // 
        // Set the speeds of the motors.
        int setMotorSpeeds(float lCmd, float rCmd);
        // 
        // Using the actual velocity update the position vector.
        int updatePos(float omegal, float omegar, float dt);
        // 
        // State variables. 
        volatile int32_t *rEncoderCount_, *lEncoderCount_;
        int32_t lastR_, lastL_;
        int32_t lastTime_;
        float w_r_, w_l_;
        float setSpeed_, setOmega_;
        float actYaw_, desYaw_;
        //
        // PIDs for motors.
        PID lVelocity_;
        PID rVelocity_;
        PID yawControl_;
        float lMotorCommand_, rMotorCommand_;
        Adafruit_DCMotor *rMotor_, *lMotor_;
        // 
        // The position of the robot from encoder counts
        float xComp_, yComp_, yaw_;
        DriveState state_;
        float upsideDown_;
};

#endif /* _DRIVE_H_ */
