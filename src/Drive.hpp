#include <stdint.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "PID.hpp"

#ifndef __DRIVE__
#define __DRIVE__

// 
// Left motor tuning.
#define LEFT_MOTOR_P 0.0
#define LEFT_MOTOR_I 0.0
#define LEFT_MOTOR_D 0.0
// 
// Right motor tuning.
#define RIGHT_MOTOR_P 0.0
#define RIGHT_MOTOR_I 0.0
#define RIGHT_MOTOR_D 0.0
// 
// The maximum and minimum motor speeds.
#define MAX_SPEED 255
#define MIN_SPEED -255
// 
// The radius of the wheels.
#define WHEEL_RADIUS 0.1524
#define WHEEL_DIAMETER (2 * 0.1524)
#pragma message("Fix this length")
#define CHASIS_LENGTH (0.3) 

// Input velocity and angle?
// https://youtu.be/aLOjNfgRdhw?t=464
// x_dot = r/2(v_r + v_l)cos(phi)
// y_dot = "            "sin(phi)
// phi_dot = R/L(v_r - v_l)
// 
// v_r = (2(v_desired) + omega_desiged*L) / 2r
// v_l = (2(v_desired) - omega_desiged*L) / 2r
class Drive {
    public:
        Drive(int32_t *rEncoderCount, int32_t *lEncoderCount, Adafruit_DCMotor *rMotor, Adafruit_DCMotor *lMotor);
        ~Drive();
        // 
        // Manage PID loops.
        int update();
        // 
        // Change the reference values desired.
        int setReference(float setSpeed, float setOmega);

    private:
        int32_t *rEncoderCount_, *lEncoderCount_;
        int32_t lastR_, lastL_;
        int32_t lastTime_;
        float v_r_, v_l_;
        //
        // PIDs for motors.
        PID lVelocity_;
        PID rVelocity_;
        int16_t lMotorCommand_, rMotorCommand_;
        Adafruit_DCMotor *rMotor_, *lMotor_;
};

#endif /* _DRIVE_H_ */
