#include <stdint.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "PID.hpp"

#ifndef __DRIVE__
#define __DRIVE__
// 
// Left motor tuning.
#define LEFT_MOTOR_P 15.0f
#define LEFT_MOTOR_I 0.0f
#define LEFT_MOTOR_D 0.0f
// 
// Right motor tuning.
#define RIGHT_MOTOR_P 15.0f
#define RIGHT_MOTOR_I 0.0f
#define RIGHT_MOTOR_D 0.0f
// 
// Yaw control tuning.
#define YAW_P 1.0f
#define YAW_I 0.0f
#define YAW_D 0.0f
// 
// The maximum and minimum motor speeds.
// Given we have a 11.1V LiPo divide 255 max by 2.
#define MAX_SPEED /*255*/(255 / 2.0f)
#define MIN_SPEED /*-255*/(-255 / 2.0f)
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
        // Manage PID loops.
        int update();
        // 
        // Change the reference values desired.
        int setReference(float setSpeed, float setOmega);
        // 
        // Stop the robot.
        int stop();
        // 
        // Reset the position vector's origin.
        int resetOrigin();
        //
        // Turn the robot 90 degrees to the left.
        int turnLeft();
        //
        // Turn the robot 90 degrees to the right.
        int turnRight();

    private:
        // 
        // Calculate the compensation for angular speeds.
        int angleComp(float omegaL, float omegaR, uint32_t dT, float &setOmega);
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
};

#endif /* _DRIVE_H_ */
