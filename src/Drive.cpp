#include "Drive.hpp"
#include "arduino.h"

Drive::Drive(int32_t *rEncoderCount, int32_t *lEncoderCount, Adafruit_DCMotor *rMotor, Adafruit_DCMotor *lMotor)
    : rEncoderCount_(rEncoderCount)
    , lEncoderCount_(lEncoderCount)
    , lastR_(0)
    , lastL_(0)
    , lastTime_(millis())
    , v_r_(0)
    , v_l_(0)
    , lVelocity_(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D)
    , rVelocity_(RIGHT_MOTOR_P, RIGHT_MOTOR_I, RIGHT_MOTOR_D)
    , rMotor_(rMotor)
    , lMotor_(lMotor)
{

}

Drive::~Drive()
{
}

int Drive::update()
{
    float calcR, calcL = 0.0f;
    int32_t curTime = millis();
    // 
    // Calculate the current speed.
    calcL = (*lEncoderCount_ - lastL_) / (curTime - lastTime_);
    calcR = (*rEncoderCount_ - lastR_) / (curTime - lastTime_);
    // 
    // Calculate the commands for the speed control.
    rMotorCommand_ += rVelocity_.getError(v_r_, calcR);
    lMotorCommand_ += lVelocity_.getError(v_l_, calcL);
    // 
    // Clamp commands.
    rMotorCommand_ = min(max(rMotorCommand_, MIN_SPEED), MAX_SPEED);
    lMotorCommand_ = min(max(lMotorCommand_, MIN_SPEED), MAX_SPEED);
    // 
    // Set commands.
    if (lMotorCommand_ < 0) {
        lMotor_->run(BACKWARD);
    }
    else {
        lMotor_->run(FORWARD);
    }
    if (rMotorCommand_ < 0) {
        rMotor_->run(BACKWARD);
    }
    else {
        rMotor_->run(FORWARD);
    }
    lMotor_->setSpeed(lMotorCommand_);
    rMotor_->setSpeed(rMotorCommand_);
    return 0;
}

int setRefence(float setSpeed, float setOmega) 
{
    v_r_ = (2 * setSpeed + setOmega * L) / WHEEL_DIAMETER;
    v_l_ = (2 * setSpeed - setOmega * L) / WHEEL_DIAMETER;
    return 0;
}




