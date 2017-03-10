#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL

#ifndef _IMU_WRAPPER_H_
#define _IMU_WRAPPER_H_

class IMU {
    public:
        IMU(uint32_t interPin);
        ~IMU();
        // 
        // Read data from the IMU.
        int read();
        // 
        // Get functions for yaw pitch and roll.
        float getYaw();
        float getPitch();
        float getRoll();
        // 
        // Accelerometer information.
        float getGlobalZ(); // in G.
        bool isUpsideDown(); // Return if the robot is upside down.
        float getGravityX();
        float getGravityY();
        float getGravityZ();

    private:
        MPU6050 mpu_;

        // Status.
        uint16_t packetSize_;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount_;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer_[64]; // FIFO storage buffer

        // Pose.
        Quaternion q_;           // [w, x, y, z]         quaternion container
        float ypr_[3] = {};      // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        VectorFloat gravity_;  // [x, y, z]
        VectorInt16 acelLocal_ = {};
        VectorInt16 acelWorld_ = {};
        VectorInt16 acel_ = {};
        float accScale_;
};

#endif /* _IMU_WRAPPER_H_ */
