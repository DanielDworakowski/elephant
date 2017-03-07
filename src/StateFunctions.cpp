#include "StateFunctions.hpp"

int StateFunctions::waitForStartButton(IMU *imu, float &yaw)
{
    uint16_t cnt;
    const uint16_t avgNum = 15;
    while (!digitalRead(PIN::startButtonPin)) {
        Serial.println("Waiting for button!");
        delay(100);
    }
    Serial.println("Button was pressed!");
    // 
    // Average the current yaw.
    for (cnt = 0; cnt < avgNum; ++cnt) {
        if (imu->read() < 0) {
            --cnt;
            continue;
        }
        yaw += imu->getYaw();
        Serial.println(imu->getYaw());
        delay(40);
    }
    Serial.println("Done averaging");
    yaw /= avgNum;
    delay(1000);
    return 0;
}

int StateFunctions::getOffPlatform(Drive *drive)
{
    float startTime = millis();
    while (millis()-startTime < DRIVE_OFF_PLATFORM_TIME) {
        drive->setReference(ROBOT_SPEED_MAX, 0.0f);
        drive->update();
        delay(20);
    }
    return 0;
}

int StateFunctions::approach(Drive *drive, VL53L0X* prox) 
{
    PID acc(ACC_P, ACC_I, ACC_D, ROBOT_SPEED_MAX / 4.0f, ROBOT_SPEED_MIN / 4.0f);
    float meas = 0, cmd = 0;
    // 
    // Monitor and control the speed using the PID and 
    do {
        do {
            meas = prox->readRangeContinuousMillimeters();
            cmd = -1.0f * acc.getCmd(WALL_SET_DIST, meas);
            drive->setReference(cmd, 0.0f);
            drive->update();
            delay(70);
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
    } while(imu->getGlobalZ() < JUMP_THRESHOLD);
    // 
    // Stop the motors.
    drive->stop();
    return 0;
}

int StateFunctions::orientForward(Drive *drive, IMU *imu, float refYaw)
{
    float cmd = 0.0;
    float curYaw = 0.0;
    PID pid(ORIENT_P, ORIENT_I, ORIENT_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    do {
        imu->read();
        curYaw = imu->getYaw();
        cmd = pid.getCmd(refYaw, curYaw);
        drive->setReference(0, -cmd);
        drive->update();
        delay(10);
        Serial.print(curYaw);
        Serial.print('\t');
        Serial.println(refYaw);
    } while (abs(curYaw - refYaw) > YAW_TOL); // Not sure if sufficient.
    drive->stop();
    return 0;
}

int StateFunctions::locateDest(Drive *drive /* Other components TBD. */)
{   
    (void) drive;
    return 0;
}