#include "StateFunctions.hpp"

int StateFunctions::waitForStartButton(IMU *imu, float &yaw)
{
    uint16_t cnt;
    const uint16_t avgNum = 15;
    while (!digitalRead(PIN::startButtonPin)) {
        imu->read();
        // Serial.print(imu->getYaw());
        // Serial.print('\t');
        // Serial.print(imu->getPitch());
        // Serial.print('\t');
        // Serial.print(imu->getRoll());
        // Serial.println('\t');
        delay(100);
    }
    Serial.println("Button was pressed!");
    // 
    // Average the current yaw.
    for (cnt = 0; cnt < avgNum; ++cnt) {
        if (imu->read() < 0) { // Time out may be needed.
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
    while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME) {
        drive->setReference(1.0 * ROBOT_SPEED_MAX, 0.0f);
        drive->update();
        delay(30);
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
        meas = prox->readRangeContinuousMillimeters();
        
        // Serial.print(meas);
        // Serial.print("\t");
        // Serial.println(WALL_SET_DIST);
        
        cmd = acc.getCmd(WALL_SET_DIST, meas);
        drive->setReference(-1.0 * cmd, 0.0f);
        drive->update();
        delay(30);
    } while (meas > WALL_JUMP_DIST);
    return 0;
}

int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, IMU *imu)
{
    // 
    // Run the motor forwards until acceleration is detected.
    jumpMotor->run(BACKWARD);
    jumpMotor->setSpeed(255);
    delay(1000);
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
    imu->read();
    float curYaw = imu->getYaw();
    PID pid(ORIENT_P, ORIENT_I, ORIENT_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    while (abs(curYaw - refYaw) > YAW_TOL) {
        imu->read();
        curYaw = imu->getYaw();
        cmd = pid.getCmd(refYaw, curYaw);
        drive->setReference(0, imu->isUpsideDown() ? -cmd : cmd);
        drive->update();
        delay(10);
        Serial.print(curYaw);
        Serial.print('\t');
        Serial.print(refYaw);
        Serial.print('\t');
        Serial.print(imu->getYaw());
        Serial.print('\t');
        Serial.print(imu->getPitch());
        Serial.print('\t');
        Serial.print(imu->getRoll());
        Serial.println('\t');
    } 
    drive->stop();
    return 0;
}

int StateFunctions::locateDest(Drive *drive, SR04 *ultrasonicLeft, RB90 *ultrasonicRight)
{   
    long leftSensor = 10000;
    long rightSensor = 10000;

    drive->setReference(ROBOT_SPEED_MAX / 4.0f, 0.0f);
    drive->update();
    delay(20);

    do {
        ultrasonicLeft->DistanceMeasure();
        ultrasonicRight->DistanceMeasure();
        leftSensor = ultrasonicLeft->microsecondsToCentimeters();
        rightSensor = ultrasonicRight->microsecondsToCentimeters();
        drive->update();
        delay(20);
    } while (leftSensor > 400 && rightSensor > 400); // assumes sensor value > 400 means nothing detected

    drive->stop();

    if (leftSensor < 400) {
        drive->turnLeft();
    }
    else {
        drive->turnRight();
    }

    return 0;
}

int StateFunctions::driveToDest(Drive *drive, IMU *imu)
{   
    imu->read();
    float oldZ = imu->getGlobalZ();
    float newZ;

    drive->setReference(ROBOT_SPEED_MAX / 4.0f, 0.0f);
    drive->update();
    delay(20);
    
    do {
        imu->read();
        newZ = imu->getGlobalZ();
        drive->update();
        delay(20);
    } while (abs(oldZ - newZ) < HEI_TOL);

    drive->stop();

    return 0;
}
