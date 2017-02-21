#include "Drive.hpp"
#include "arduino.h"

Drive::Drive(int32_t *rEncoderCount, int32_t *lEncoderCount, Adafruit_DCMotor *rMotor, Adafruit_DCMotor *lMotor)
    : rEncoderCount_(rEncoderCount)
    , lEncoderCount_(lEncoderCount)
    , lastR_(0)
    , lastL_(0)
    , lastTime_(millis())
    , v_r_(0.0f)
    , v_l_(0.0f)
    , lVelocity_(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, MAX_SPEED, MIN_SPEED)
    , rVelocity_(RIGHT_MOTOR_P, RIGHT_MOTOR_I, RIGHT_MOTOR_D, MAX_SPEED, MIN_SPEED)
    , lMotorCommand_(0.0)
    , rMotorCommand_(0.0)
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
    float deltaT = (curTime - lastTime_) / 1000.0;
    if (deltaT <= 0) {
        return -1;
    }
    // 
    // Calculate the current speed.
    calcL = TO_OMEGA(*lEncoderCount_ - lastL_) / (deltaT);
    calcR = TO_OMEGA(*rEncoderCount_ - lastR_) / (deltaT);
    // 
    // Update state variables.
    lastL_ = *lEncoderCount_;
    lastR_ = *rEncoderCount_;
    lastTime_ = curTime;
    // 
    // Calculate the commands for the speed control.
    rMotorCommand_ = rVelocity_.getError(v_r_, calcR);
    lMotorCommand_ = lVelocity_.getError(v_l_, calcL);
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

int Drive::setReference(float setSpeed, float setOmega) 
{
    v_r_ = (2 * setSpeed + setOmega * CHASIS_LENGTH) / WHEEL_DIAMETER;
    v_l_ = (2 * setSpeed - setOmega * CHASIS_LENGTH) / WHEEL_DIAMETER;
    return 0;
}




