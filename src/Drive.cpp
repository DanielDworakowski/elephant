#include "Drive.hpp"
#include "Arduino.h"

Drive::Drive(volatile int32_t *rEncoderCount, volatile int32_t *lEncoderCount, Adafruit_DCMotor *rMotor, Adafruit_DCMotor *lMotor)
    : rEncoderCount_(rEncoderCount)
    , lEncoderCount_(lEncoderCount)
    , lastR_(0)
    , lastL_(0)
    , lastTime_(millis())
    , w_r_(0.0f)
    , w_l_(0.0f)
    , lVelocity_(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, MAX_SPEED, MIN_SPEED)
    , rVelocity_(RIGHT_MOTOR_P, RIGHT_MOTOR_I, RIGHT_MOTOR_D, MAX_SPEED, MIN_SPEED)
    , lMotorCommand_(0.0f)
    , rMotorCommand_(0.0f)
    , rMotor_(rMotor)
    , lMotor_(lMotor)
    , xComp_(0.0f)
    , yComp_(0.0f)
    , yaw_(0.0f)
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
    rMotorCommand_ = -1 * rVelocity_.getCmd(w_r_, calcR);
    lMotorCommand_ = -1 * lVelocity_.getCmd(w_l_, calcL);
    // Serial.print(calcL);
    // Serial.print("\t");
    // Serial.println(calcR);
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
    lMotor_->setSpeed(abs(lMotorCommand_));
    rMotor_->setSpeed(abs(rMotorCommand_));
    return 0;
}

int Drive::setReference(float setSpeed, float setOmega) 
{
    w_r_ = (2 * setSpeed + setOmega * CHASIS_LENGTH) / WHEEL_DIAMETER;
    w_l_ = (2 * setSpeed - setOmega * CHASIS_LENGTH) / WHEEL_DIAMETER;
    return 0;
}

int Drive::stop()
{
    lMotor_->setSpeed(0);
    rMotor_->setSpeed(0);
    setReference(0,0);
    return 0;
}

// 
// Not guarunteed to work.
int Drive::updatePos(float omegal, float omegar, float dt)
{
    float vec = 0, angle = 0, xCompLocal = 0, yCompLocal = 0;
    // 
    // Find the radius of curvature driven.
    float vel = (omegal + omegar) * WHEEL_RADIUS / 2.0f;
    float omegaG = WHEEL_RADIUS * (omegar - omegal) / CHASIS_LENGTH;
    // 
    // Find the radius of curvature and angle change.
    float angleTraversed = omegaG * dt;
    // 
    // If there is a significant change in angle calculate as if a circle.
    if (omegaG > 0.000001) { // Max 3cm error over 2m.
        float radCurv = vel / omegaG;
        // 
        // Cosine law to get the vector length.
        vec = sqrt(2) * radCurv * sqrt(1 - cos(angle));
        angle = 90.0f - angleTraversed / 2.0f;
    }
    // 
    // It basically drove straight 
    else {
        vec = vel * dt;
        angle = 90;
    }

    xCompLocal = vec * cos(90 - angle/2.0f);
    yCompLocal = vec * sin(90 - angle/2.0f);
    xComp_ += xCompLocal*cos(yaw_) + yCompLocal*sin(yaw_);
    yComp_ += -xCompLocal*sin(yaw_) + yCompLocal*cos(yaw_);
    yaw_ += angle;
    return 0;
}


