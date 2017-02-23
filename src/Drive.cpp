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
    rMotorCommand_ = rVelocity_.getCmd(v_r_, calcR);
    lMotorCommand_ = lVelocity_.getCmd(v_l_, calcL);
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


