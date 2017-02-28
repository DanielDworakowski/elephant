#include "StateFunctions.hpp"

int StateFunctions::waitForStartButton(IMU *imu, float &yaw)
{
    (void) imu;
    (void) yaw;
    while (!digitalRead(PIN::startButtonPin)) {
        delay(100);
    }
    delay(1000);
    return 0;
}

int StateFunctions::getOffPlateform(Drive *drive)
{
    float startTime = millis();
    while (millis()-startTime < DRIVE_OFF_PLATFORM_TIME) {
        drive->setReference(ROBOT_SPEED_MAX, 0.0f);
        drive->update();
    }
    return 0;
}

int StateFunctions::approach(Drive *drive, VL53L0X* prox) 
{
    PID acc(ACC_P, ACC_I, ACC_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    float meas = 0, cmd = 0;
    // 
    // Monitor and control the speed using the PID and 
    do {
        do {
            meas = prox->readRangeContinuousMillimeters();
            cmd = -1.0f * acc.getCmd(WALL_SET_DIST, meas);
            drive->setReference(cmd, 0.0f);
            drive->update();
        } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
        // 
        // Stop and ensure no drift. 
        drive->stop();
        delay(10);
    } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
    return 0;
}

int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, IMU *imu)
{
    // 
    // Run the motor forwards until acceleration is detected.
    jumpMotor->run(FORWARD);
    jumpMotor->setSpeed(255);
    // 
    // Wait for acceleration upwards.
    do {
        imu->read();
    } while(imu->getGlobalZ() < JUMP_THRESHOLD);
    // 
    // Stop the motor.
    jumpMotor->setSpeed(0);
    return 0;
}

int StateFunctions::inAir(Drive *drive, IMU *imu)
{
    float cmd = 0.0;
    PID pid(AIR_P, AIR_I, AIR_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    do {
        imu->read();
        cmd = pid.getCmd(0, imu->getPitch());
        drive->setReference(cmd, 0);
        drive->update();
        Serial.print(imu->getPitch());
        Serial.print("\t");
        Serial.println(cmd);
    } while(imu->getGlobalZ() < JUMP_THRESHOLD);
    // 
    // Stop the motors.
    drive->stop();
    return 0;
}

int StateFunctions::orientForward(Drive *drive, IMU *imu, float yaw)
{
    (void) drive;
    (void) imu;
    (void) yaw;
    return 0;
}

int StateFunctions::locateDest(Drive *drive /* Other components TBD. */)
{   
    (void) drive;
    return 0;
}