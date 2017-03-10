#include "IMU_Wrapper.hpp"
#include "PinDefines.h"

// Indicate that there is data available on the IMU.
volatile bool gMpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    gMpuInterrupt = true;
}

IMU::IMU(uint32_t interPin) 
{
    bool dmpReady = false;
    uint8_t devStatus;
    // 
    // Force pin to be in interrupt. 
    pinMode(interPin, INPUT);
    mpu_.initialize();
    // 
    // Verify connection.
    Serial.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(2);
    devStatus = mpu_.dmpInitialize();
    // 
    // Set calibrations.
    // Sensor readings with offsets:    -9  0   16381   0   0   -2
    // Your offsets:   569 1331    5521    28  35  25
    mpu_.setXAccelOffset(569);
    mpu_.setYAccelOffset(1331);
    mpu_.setZAccelOffset(5521);
    mpu_.setXGyroOffset(28);
    mpu_.setYGyroOffset(35);
    mpu_.setZGyroOffset(25);
    // 
    // Make sure it worked (returns 0 if so).
    if (devStatus == 0) {
        mpu_.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(interPin), dmpDataReady, RISING);
        dmpReady = true;
        packetSize_ = mpu_.dmpGetFIFOPacketSize();
    } 
    else {
        // 
        // 1 = initial memory load failed.
        // 2 = DMP configuration updates failed.
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    while (!dmpReady) {
        Serial.println("UNABLE TO START IMU! WILL NOT CONTINUE\n");
    }
    // 
    // Read the full scale range of the accelerometer. 
    switch (mpu_.getFullScaleAccelRange()) {
        case 0:
            accScale_ = 8192.0f;
            break;
        case 1:
            accScale_ = 4096.0f;
            break;
        case 2:
            accScale_ = 2048.0f;
            break;
        case 3:
            accScale_ = 1024.0f;
            break;
    }
}

IMU::~IMU() 
{
}

int IMU::read() 
{
    uint8_t mpuIntStatus;

    while (!gMpuInterrupt && fifoCount_ < packetSize_) {
        fifoCount_ = mpu_.getFIFOCount();
    }
    // 
    // Reset interrupt flag and get status.
    gMpuInterrupt = false;
    mpuIntStatus = mpu_.getIntStatus();
    fifoCount_ = mpu_.getFIFOCount();
    // 
    // check for overflow.
    if ((mpuIntStatus & 0x10) || fifoCount_ == 1024) {
        mpu_.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return -1;
    } 

    if (mpuIntStatus & 0x02) {
        // 
        // Read from the fifo until there isnt anything left.
        do {
            // 
            // Wait for the fifo to obtain data.
            while (fifoCount_ < packetSize_) {
                fifoCount_ = mpu_.getFIFOCount();
            }
            mpu_.getFIFOBytes(fifoBuffer_, packetSize_);
            // 
            // Reduce the fifo count. 
            fifoCount_ -= packetSize_;
        } while (fifoCount_ > 2 * packetSize_);

        mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
        mpu_.dmpGetGravity(&gravity_, &q_);
        mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity_);
        mpu_.dmpGetAccel(&acel_, fifoBuffer_);
        mpu_.dmpGetLinearAccel(&acelLocal_, &acel_, &gravity_);
        mpu_.dmpGetLinearAccelInWorld(&acelWorld_, &acelLocal_, &q_);
    } 
    else {
        return -1;
    }

    return 0;
}

float IMU::getYaw() 
{
    if (ypr_[0] < 0) {
        return 360 + (ypr_[0] * 180.0f / M_PI);
    }
    return ypr_[0] * 180.0f / M_PI;
}

float IMU::getPitch() 
{
    // if (ypr_[1] < 0) {
    //     return 360 + (ypr_[1] * 180.0f / M_PI);
    // }
    return ypr_[1] * 180.0f / M_PI;
}

float IMU::getRoll() 
{
    if (ypr_[2] < 0) {
        return 360 + (ypr_[2] * 180.0f / M_PI);
    }
    return ypr_[2] * 180.0f / M_PI;
}

float IMU::getGlobalZ()
{
    return acelWorld_.z / accScale_;
}

bool IMU::isUpsideDown()
{
    return (gravity_.z < 0.0f);
}

float IMU::getGravityX()
{
    return gravity_.x;
}

float IMU::getGravityY()
{
    return gravity_.y;
}

float IMU::getGravityZ()
{
    return gravity_.z;
}
