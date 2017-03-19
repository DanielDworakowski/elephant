#include "StateFunctions.hpp"

#define DEBUG
#ifdef DEBUG

#define dprint(x) do { Serial.print(x); } while (0)
#define dprintln(x) do { Serial.println(x); } while (0)

#else

#define dprint(x) 
#define dprintln(x) 

#endif 

/* 
 * Parameters: none
 * Waits for button press, then a certain amount of time before continuing.
 */
int StateFunctions::waitForStartButton(Adafruit_DCMotor *jumpMotor)
{
    while (!digitalRead(PIN::startButtonPin)) {
        // 
        // Shaft alignment code.
        if (!digitalRead(PIN::alignButtonPin)) {
            jumpMotor->run(BACKWARD);
            jumpMotor->setSpeed(255.0f / 4.0f);
            delay(50);
            jumpMotor->setSpeed(0);
            Serial.println("Align button is pressed!");
        }
    }
    return 0;
}

/* 
 *  Parameters: pointer to IMU interface instance, address of yaw storage variable.
 *  Takes a number of samples of the current yaw and saves the average to the pointer.
 */
int StateFunctions::sampleYaw(IMU *imu, float &yaw)
{
    uint16_t cnt;
    const uint16_t avgNum = 15;
    dprintln("//// State - enter sampleYaw.");
    // 
    // Give 2 seconds for the IMU to stablize, as well as to get out of the way.
    delay(IMU_STABLIZE_TIME);
    // 
    // Average the current yaw.
    for (cnt = 0; cnt < avgNum; ++cnt) {
        if (imu->read() < 0) { // Time out may be needed.
            --cnt;
            continue;
        }

        yaw += imu->getYaw();
        delay(40);
    }
    yaw /= avgNum;
    dprintln("//// State - exit sampleYaw.");
    return 0;
}

int StateFunctions::driveStraight(Drive *drive, float speed, uint32_t timeMs)
{
    uint32_t startTime = millis();
    drive->setReference(speed, 0);
    do {
        drive->update();
        delay(30);
    } while((millis() - startTime) < timeMs);
    return 0;
}

int StateFunctions::approach2(Drive *drive, VL53L0X* prox) 
{
    const float numSteps = 20.0f;
    const int minTriggerTime = 3000;
    uint32_t minTriggerStart = millis();
    float step = 0;
    float startTime;
    float meas;
    drive->reset(30);
    for (step = 0; step < numSteps; ++step) {
        meas = prox->readRangeContinuousMillimeters();
        startTime = millis();
        while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME / numSteps) {
            drive->setReference(-ROBOT_SPEED_MAX * (step / numSteps), 0.0f);
            drive->update();
            meas = prox->readRangeContinuousMillimeters();
            if (meas < WALL_JUMP_DIST && ((millis() - minTriggerStart) > minTriggerTime)) {
                break;
            }
        }
    }
    do {
        meas = prox->readRangeContinuousMillimeters();
        drive->update();
    } while (meas > WALL_JUMP_DIST);
    return 0;
}

int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, IMU *imu, Drive* drive)
{
    // 
    // Run the motor forwards until acceleration is detected.
    jumpMotor->run(BACKWARD);
    jumpMotor->setSpeed(255);
    delay(1000);
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
        cmd = -1 * pid.getCmd(refYaw, curYaw);
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
        Serial.print('\t');
        Serial.println(cmd);

    } 
    drive->stop();
    return 0;
}

/*
 *  Parameters: pointer to drive motor interface instance, pointer to left and right ultrasonic interface.
 *  Continues driving until either of the ultrasonics detect a large difference in measurement.
 *  Then the robot turns in that direction.
 */
int StateFunctions::locateDest(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight)
{   
    long leftData[3];
    long leftCurrentData;
    long leftLastData;
    long rightData[3];
    long rightCurrentData;
    long rightLastData;
    int initIterations = 10;
    uint32_t startTime = millis();
    int32_t sleepTime = 0;

    dprintln("//// State - enter locateDest.");
    // 
    // The first few measurements of the sensor usually is very off.
    // "Warm up" the sensor by measuring a few times.
    while (--initIterations > 3) {
        ultrasonicLeft->distanceMeasure();
        ultrasonicRight->distanceMeasure();

        delay(30);
    }
    // 
    // Take initial measurements to fill data buffer.
    while (--initIterations >= 0) {
        ultrasonicLeft->distanceMeasure();
        leftData[2 - initIterations] = ultrasonicLeft->microsecondsToCentimeters();

        ultrasonicRight->distanceMeasure();
        rightData[2 - initIterations] = ultrasonicRight->microsecondsToCentimeters();
    }
    // 
    // Compute median.
    leftCurrentData = max(min(leftData[0], leftData[1]), min(max(leftData[0], leftData[1]), leftData[2]));
    rightCurrentData = max(min(rightData[0], rightData[1]), min(max(rightData[0], rightData[1]), rightData[2]));
    // 
    // Start moving.
    drive->setReference(ROBOT_SPEED_MAX / 8.0, 0.0f);
    drive->update();
    do {
        startTime = millis();
        // 
        // Measure then evaluate delta.
        ultrasonicLeft->distanceMeasure();
        leftData[0] = leftData[1];
        leftData[1] = leftData[2];
        leftData[2] = ultrasonicLeft->microsecondsToCentimeters();
        // 
        // Save data.
        leftLastData = leftCurrentData;
        leftCurrentData = max(min(leftData[0], leftData[1]), min(max(leftData[0], leftData[1]), leftData[2]));
        // 
        // Update the drive if needed.
        if ((millis() - startTime) > DRIVE_SLEEP_TIME){
            drive->update();
        }
        // 
        // Measure and evaluate the right sensor.
        ultrasonicRight->distanceMeasure();
        rightData[0] = rightData[1];
        rightData[1] = rightData[2];
        rightData[2] = ultrasonicRight->microsecondsToCentimeters();
        // 
        // Save data.
        rightLastData = rightCurrentData;
        rightCurrentData = max(min(rightData[0], rightData[1]), min(max(rightData[0], rightData[1]), rightData[2]));
        drive->update();
        // 
        // Calculate how much to sleep to keep controller stable.
        sleepTime = DRIVE_SLEEP_TIME - (millis() - startTime);
        sleepTime = sleepTime > 0 ? sleepTime : 0;
        delay(sleepTime);
    } while (abs(leftCurrentData - leftLastData) < ULTRASONIC_DELTA_TOLERANCE && abs(rightCurrentData - rightLastData) < ULTRASONIC_DELTA_TOLERANCE);

    dprint("Pole found ");
    drive->stop();
    
    if (abs(leftCurrentData - leftLastData) > ULTRASONIC_DELTA_TOLERANCE) {
        dprintln("on left side.");
        drive->turnTheta(90);
    }
    else {
        dprintln("on right side.");
        drive->turnTheta(-90);
    }

    dprintln("//// State - exit locateDest.");
    return 0;
}

int StateFunctions::driveToDest(Drive *drive, IMU *imu)
{   
    imu->read();
    float oldZ = imu->getGlobalZ();
    float newZ;

    drive->setReference(ROBOT_SPEED_MAX / 4.0, 0.0f);
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
