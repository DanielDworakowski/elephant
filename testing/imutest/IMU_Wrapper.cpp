#include "IMU_Wrapper.hpp"

// Indicate that there is data available on the IMU.
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

IMU::IMU(uint32_t interPin) 
{
    bool dmpReady = false;
    uint8_t devStatus;

    // Force pin to be in interrupt. 
    pinMode(interPin, INPUT);
    mpu_.initialize();

    // Verify connection.
    Serial.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(2);
    devStatus = mpu_.dmpInitialize();

    // Set calibrations.
    mpu_.setXGyroOffset(-1);
    mpu_.setYGyroOffset(-1);
    mpu_.setZGyroOffset(-1);
    mpu_.setZAccelOffset(1788);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu_.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(interPin), dmpDataReady, RISING);
        dmpReady = true;
        packetSize_ = mpu_.dmpGetFIFOPacketSize();
    } 
    else {

        // 1 = initial memory load failed.
        // 2 = DMP configuration updates failed.
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    while (!dmpReady) {
        Serial.println("UNABLE TO START IMU! WILL NOT CONTINUE\n");
    }
}

IMU::~IMU() 
{

}

int IMU::read() 
{
    uint8_t mpuIntStatus;
    VectorFloat gravity;    // [x, y, z].
    int16_t gyro[3];        // [x, y, z].

    while (!mpuInterrupt && fifoCount_ < packetSize_) {

    }

    // Reset interrupt flag and get status.
    mpuInterrupt = false;
    mpuIntStatus = mpu_.getIntStatus();
    fifoCount_ = mpu_.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount_ == 1024) {
        mpu_.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return -1;
    } 

    if (mpuIntStatus & 0x02) {
        // Read from the fifo until there isnt anything left.
        do {
            // Wait for the fifo to obtain data.
            while (fifoCount_ < packetSize_) {
                fifoCount_ = mpu_.getFIFOCount();
            }
            mpu_.getFIFOBytes(fifoBuffer_, packetSize_);
            
            // Reduce the fifo count. 
            fifoCount_ -= packetSize_;
        } while (fifoCount_ > 2 * packetSize_);

        mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
        mpu_.dmpGetGyro(gyro, fifoBuffer_);
        mpu_.dmpGetGravity(&gravity, &q_);
        mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity);

        Serial.print(millis());
        Serial.print(" ");
        Serial.print(ypr_[0] * 180/M_PI);
        Serial.print(" ");
        Serial.print(ypr_[1] * 180/M_PI);
        Serial.print(" ");
        Serial.print(ypr_[2] * 180/M_PI);
        Serial.print("\n");
    }

    return 0;
}









