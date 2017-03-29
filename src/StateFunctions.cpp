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
    dprint("Setting yaw value to ");
    dprintln(yaw);
    dprintln("//// State - exit sampleYaw.");
    return 0;
}

int StateFunctions::driveStraight(Drive *drive, float speed, uint32_t timeMs)
{
    // drive->reset(30);
    uint32_t startTime = millis();
    drive->setReference(speed, 0);
    do {
        drive->update();
        delay(30);
    } while((millis() - startTime) < timeMs);
    return 0;
}

int StateFunctions::curveHelper(Drive *drive, uint32_t turnTimer)
{
    const float omega = -60.0f;
    const uint32_t straightTime = 3000; 
    const uint32_t turnTime = 500;
    // 
    // Setup the turning profile. 
    if (millis() - turnTimer < straightTime) {
        drive->setOmega(0);
    }
    else if (millis() - turnTimer - straightTime < turnTime) {
        drive->setOmega(omega);
    }
    else {
        drive->goStraight();
    }
    return 0;
}

int StateFunctions::approachAndStop(Drive *drive, VL53L0X* prox)
{
    float meas[3];
    uint32_t startTime = 0;
    uint32_t sleepTime = 0;
    const uint32_t ignoreMeasTime = 500;
    //
    // Fill up the median filter. 
    for (int x = 0; x < 3; ++x) {
        meas[x] = prox->readRangeContinuousMillimeters();
    }
    float measDist = max(min(meas[0], meas[1]), min(max(meas[0], meas[1]), meas[2]));
    // 
    // Set the reference to be constant. 
    drive->setReference(-ROBOT_SPEED_MAX / 3.5f, 0);
    startTime = millis();
    while (millis() - startTime < ignoreMeasTime) {
        drive->update();
        delay(30);
    }
    // 
    // Begin checking for the wall, assumed to be in steady state and not wobling.
    while (measDist > WALL_JUMP_DIST_0_VEL) {
        startTime = millis();
        meas[0] = meas[1];
        meas[1] = meas[2];
        meas[2] = prox->readRangeContinuousMillimeters();
        measDist = max(min(meas[0], meas[1]), min(max(meas[0], meas[1]), meas[2]));
        drive->update();
        Serial.print("Update time: ");
        Serial.println(millis() - startTime);
        // sleepTime = DRIVE_SLEEP_TIME - (millis() - startTime);
        // sleepTime = sleepTime > 0 ? sleepTime : 0;
        // delay(sleepTime);
        // delay(30);
    }
    drive->stop();
    delay(3000);
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
    // drive->reset(30);
    //
    // Acceleration by the number of steps equally. 
    for (step = 0; step < numSteps; ++step) {
        startTime = millis();
        drive->setReference(-ROBOT_SPEED_MAX * (step / numSteps), 0);
        while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME / numSteps) {
            drive->update();
            meas = prox->readRangeContinuousMillimeters();
            if (meas < WALL_JUMP_DIST_INIT_VEL && ((millis() - minTriggerStart) > minTriggerTime)) {
                break;  
            }
        }
    }
    // 
    // We are done accelerating make sure the drive is going in steady state.
    do {
        meas = prox->readRangeContinuousMillimeters();
        drive->update();
    } while (meas > WALL_JUMP_DIST_INIT_VEL);
    return 0;
}

int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, Drive* drive)
{
    const uint32_t motorRunTime = 500;
    uint32_t startMotorRun = 0;
    // 
    // Run the motor forwards until acceleration is detected.
    drive->reset(30);
    jumpMotor->run(BACKWARD);
    jumpMotor->setSpeed(255);
    delay(1100);
    drive->setReference(ROBOT_SPEED_MAX, 0.0f);
    startMotorRun = millis();
    do {
        drive->update();
    } while (millis() - startMotorRun < motorRunTime);
    jumpMotor->setSpeed(0);
    drive->stop();
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

int StateFunctions::orientForwardIMU(Drive *drive, IMU *imu, float refYaw)
{
    dprintln("//// State - enter orientForwardIMU.");
    drive->setTurn();
    float cmd = 0.0;
    while (imu->read() != 0) {};
    float curYaw = imu->getYaw();
    PID pid(ORIENT_P, ORIENT_I, ORIENT_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    while (abs(curYaw - refYaw) > YAW_TOL) {
        if (imu->read() != 0) {
            continue;
        }
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
    dprintln("//// State - exit orientForwardIMUx.");
    drive->stop();
    return 0;
}

int StateFunctions::orient(Drive *drive, VL53L0X *prox, Ultrasonic* ultrasonicL, Ultrasonic* ultrasonicR)
{
    //
    // Check if one side is too close to rotate and treat it as the wall?
    // ^ Would likely need a proper set distance for this to avoid dying.  
    // make assumption about minimal wall distance and rotate until there is a tolerance
    // follow the wall as required.
    Ultrasonic *poleSensor;
    Ultrasonic *wallSensor;
    bool isUpsideDown;
    int initIterations = 10;

    long wallData[WALL_ORIENT_READ_COUNT];
    long wallCurrentData;

    long distanceToWall;
    long numberOfTurns = 360.0 / WALL_ORIENT_ANGLE;
    int turnCount = 0;
    int readCount = 0;
    //
    // Check if the robot is upside down. 
    isUpsideDown = prox->isUpsideDown();
    //
    //Determine which sensor is looking for pole.
    if (isUpsideDown) {
        poleSensor = ultrasonicL;
        wallSensor = ultrasonicR;
    }
    else {
        poleSensor = ultrasonicR;
        wallSensor = ultrasonicL;
    }

    // 
    // The first few measurements of the sensor usually is very off.
    // "Warm up" the sensor by measuring a few times.
    while (--initIterations > 3) {
        poleSensor->distanceMeasure();
        wallSensor->distanceMeasure();

        delay(30);
    }

    for (turnCount = 0; turnCount < numberOfTurns; turnCount++) {
        drive->turnTheta(WALL_ORIENT_ANGLE);

        for(readCount = 0; readCount < WALL_ORIENT_READ_COUNT; readCount++) {
            wallSensor->distanceMeasure();
            wallData[readCount] = wallSensor->microsecondsToCentimeters();
        }
        wallCurrentData = max(min(wallData[0], wallData[1]), min(max(wallData[0], wallData[1]), wallData[2]));
        distanceToWall = min(wallCurrentData, distanceToWall);
    }

    do {
        drive->turnTheta(WALL_ORIENT_ANGLE);

        for(readCount = 0; readCount < WALL_ORIENT_READ_COUNT; readCount++) {
            wallSensor->distanceMeasure();
            wallData[readCount] = wallSensor->microsecondsToCentimeters();
        }
        wallCurrentData = max(min(wallData[0], wallData[1]), min(max(wallData[0], wallData[1]), wallData[2]));
    } while (abs(wallCurrentData - distanceToWall) < WALL_ORIENT_TOLERANCE);

    return 0;
}

int StateFunctions::checkUpsideDown(Drive *drive, VL53L0X *prox)
{
    drive->setUpsideDown(prox->isUpsideDown());
    drive->reset(30);
    return 0;
}

bool atDesitinationHelper(IMU* imu, bool isUpsideDown)
{   
    float destinationYTol = -0.25;
    // 
    // Keep reading until success.
    while (imu->read() != 0) {};
    if (!isUpsideDown) {
        if (imu->getGlobalY() < destinationYTol) {
            // return true;
        }
    }
    else {
        if (imu->getGlobalY() > -destinationYTol) {
            // return true;
        } 
    }
    return false;
}

int StateFunctions::locateDriveHelper(Drive *drive, float curDist, float setDist)
{
    (void) curDist;
    (void) setDist;
    // static PID distPID(LOCATE_P, LOCATE_I, LOCATE_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    // float cmd = 0;
    // cmd = -distPID.getCmd(setDist, curDist);
    // drive->setOmega(cmd);
    drive->update();
    // Serial.print("curDist: ");
    // Serial.print(curDist);
    // Serial.print(" setDist: ");
    // Serial.print(setDist);
    // Serial.print(" cmd: ");
    // Serial.println(cmd);
    return 0;
}

/*
 *  Parameters: pointer to the ultrasonic sensor interface instance.
 *  Reads ultrasonic sensor and returns the distance value in centimeters.
 */
long readUltrasonic(Ultrasonic *sensor) 
{
    sensor->distanceMeasure();
    return sensor->microsecondsToCentimeters();
}

/*
 *  Parameters: pointer to the ultrasonic sensor interface instance, pointer to the data storage array.
 *  Reads ultrasonic sensor and passes it through the median filter, and returns the filtered value in centimeters.
 */
long readUltrasonic(Ultrasonic *sensor, long *data)
{
    data[0] = data[1];
    data[1] = data[2];
    data[2] = readUltrasonic(sensor);

    return max(min(data[0], data[1]), min(max(data[0], data[1]), data[2]));
}

/*
 *  Parameters: pointer to the ultrasonic sensor interface instance, pointer to the data storage array.
 *  Reads ultrasonic 3 times to fill the data buffer, then returns the median filtered value in centimeters.
 */
long flushUltrasonicDataBuffer(Ultrasonic *sensor, long *data)
{
    readUltrasonic(sensor, data);
    // delay(30);
    readUltrasonic(sensor, data);
    // delay(30);
    return readUltrasonic(sensor, data);
}

/*
 *  Parameters: pointer to drive motor interface instance, pointer to left and right ultrasonic interface.
 *  Continues driving until either of the ultrasonics detect a large difference in measurement.
 *  Then the robot turns and runs perpenticular to the ramp.
 */
int StateFunctions::locateDestAlternative(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight, VL53L0X *prox, uint32_t &poleDist)
{   
    Ultrasonic *sensor;
    bool isUpsideDown;
    long data[3] = {0, 0, 0};
    long currentData;
    long lastData;
    int initIterations = 10;
    uint32_t startTime = millis();
    int32_t sleepTime = 0;
    int confirmationCheckCount = POLE_CONFIRMATION_CHECK_COUNT_ALTERNATIVE;
    int confirmationCheckPassed = 0;
    const int32_t stepSize = 10;




    uint32_t wallMaxDist = 50;
    uint32_t turnCount = 0;



    dprintln("//// State - enter locateDest.");

    //
    // Check if the robot is upside down. 
    isUpsideDown = prox->isUpsideDown();
    //
    //Determine which sensor is looking for pole.
    if (isUpsideDown) {
        sensor = ultrasonicRight;
    }
    else {
        sensor = ultrasonicLeft;
    }
    // 
    // The first few measurements of the sensor usually is very off.
    // "Warm up" the sensor by measuring a few times.
    while (--initIterations > 0) {
        sensor->distanceMeasure();
        delay(30);
    }
    //
    // Get current data.
    currentData = flushUltrasonicDataBuffer(sensor, data);
    //
    // Turn until it is parallel to some wall.
    do {
        drive->reset(30);
        drive->turnTheta(stepSize);
        //
        // Have to flush the buffer again since this is facing a different direction now.
        // Want to apply the median filter only on values measured at this direction.
        lastData = currentData;
        currentData = flushUltrasonicDataBuffer(sensor, data);

        ++turnCount;
        wallMaxDist += static_cast<uint32_t>(turnCount / 36) * 25;
        if (turnCount >= 36) {
            turnCount = 0;
        }
    } while (currentData - lastData < 0 || currentData > wallMaxDist);

    // Turn back and run the minimization algorithm backwards to ensure we are actually facing the minimum distance point.
    while (currentData - lastData > 0 || currentData > wallMaxDist) {
        drive->reset(30);
        drive->turnTheta(-stepSize);
        //
        // Flushing the buffer for the same reason above.
        lastData = currentData;
        currentData = flushUltrasonicDataBuffer(sensor, data);
    } 
    // 
    // Undo the last turn. Should be parallel to the wall now.
    drive->reset(30);
    drive->turnTheta(stepSize);
    //
    // Turn perpenticular to the wall.
    drive->reset(30);
    if (!isUpsideDown) {
        drive->turnTheta(-85); // 92 to accomodate for backlash.
    }
    else {
        drive->turnTheta(-85);
    }
    drive->reset(30);
    startTime = millis();
    drive->setReference(-ROBOT_SPEED_MAX / 4.0f, 0);
    while (millis() - startTime < 3000) {
        drive->update();
        delay(30);
    }
    drive->reset(30);
    //
    // This outer while loop runs until the pole has been confirmed to be found.
    // If the confirmation check fails, it will loop back to going forward and searching (inner loop).
    do {
        // 
        // Start moving.
        drive->setReference(ROBOT_SPEED_MAX / 4.0, 0.0f);
        drive->update();
        //
        // Fill data buffer with latest data by flushing the buffer.
        lastData = currentData;
        currentData = flushUltrasonicDataBuffer(sensor, data);

        //
        // This inner loop drives and takes measurements until something that appears to be the pole is detected.
        do {
            startTime = millis();
            // 
            // Measure data and shift down data buffer.
            lastData = currentData;
            currentData = readUltrasonic(sensor, data);
            //
            // If the value is larger than the noise ceiling, disregard it and use previous value.
            if (currentData > POLE_NOISE_CEIL_ALTERNATIVE) {
                currentData = lastData;
            }
            // 
            // Left right ultrasonics. 
            dprint("Pole: ");
            dprintln(currentData);
            drive->update();
            //
            // Calculate how much to sleep to keep controller stable.
            sleepTime = DRIVE_SLEEP_TIME - (millis() - startTime);
            sleepTime = sleepTime > 0 ? sleepTime : 0;
            delay(sleepTime);
        } while (currentData - lastData > -POLE_DELTA_TOLERANCE_ALTERNATIVE);
        //
        // Do confirmation check on the pole measurement.
        drive->stop();
        confirmationCheckCount = POLE_CONFIRMATION_CHECK_COUNT_ALTERNATIVE;
        confirmationCheckPassed = 0;

        while (confirmationCheckCount > 0) {
            // 
            // Check if the measurement was not a mistake through a tolerance.
            // Note that this uses raw data instead of median filter as it is verifying the raw data.
            long meas = readUltrasonic(sensor, data);
            dprint("Meas check: ");
            dprintln(meas);
            if (abs(meas - currentData) <= POLE_CONFIRMATION_TOLERANCE_ALTERNATIVE) {
                confirmationCheckPassed++;
            }
            confirmationCheckCount--;
        }
    } while (confirmationCheckPassed < POLE_CONFIRMATION_CHECK_COUNT_ALTERNATIVE - POLE_CONFIRMATION_CHECK_FAIL_TOLERANCE_ALTERNATIVE);
    // 
    // May need to implement something to check if crashed. Also may need better method of handling false negative, maybe backing up.
    dprintln("Pole found.");
#pragma message("Check if this is needed.")
    drive->reset(30);
    drive->turnTheta(90);
    // 
    // Save the current pole distance to pass to the next state.
    poleDist = currentData;
    dprintln("//// State - exit locateDest.");
    return 0;
}

int StateFunctions::driveToDest(Drive *drive, VL53L0X* prox)
{   
    (void) prox;
    //
    // Check whether the robot is upside down. 
    // bool isUpsideDown = prox->isUpsideDown();
    // 
    // Start moving the robot. 
    drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
    drive->update();
    while (1) {
        drive->update();
        delay(30);
    }
    return 0;
}

int StateFunctions::driveToDestIMU(Drive *drive, IMU *imu, VL53L0X* prox)
{   
    //
    // Check whether the robot is upside down. 
    bool isUpsideDown = prox->isUpsideDown();
    // 
    // Start moving the robot. 
    drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
    drive->update();
    delay(20);
    // 
    // Continue until a bump is detected. 
    do {
        while (imu->read() != 0) {};
        drive->update();
        dprint(" newX: ");
        dprint(imu->getGlobalX());
        dprint(" newY: ");
        dprintln(imu->getGlobalY());
        delay(30);
    } while (!atDesitinationHelper(imu, isUpsideDown));
    drive->stop();
    return 0;
}

long readUltrasonic(Ultrasonic *ultrasonic, int numberOfReadings)
{
    int readCount = 0;
    long reading[numberOfReadings];

    for(readCount = 0; readCount < numberOfReadings; readCount++) {
        ultrasonic->distanceMeasure();
        reading[readCount] = ultrasonic->microsecondsToCentimeters();
    }
    return max(min(reading[0], reading[1]), min(max(reading[0], reading[1]), reading[2]));
}

// int StateFunctions::poleSearchGeneral(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight, VL53L0X *prox)
// {
//     Ultrasonic *temp;
//     Ultrasonic *tracker;
//     float trackerValue;
//     float previousTrackerValue;
//     int initIterations = 10;
//     int multiplier = 1;
//     int numberOfSteps = 3;
//     int turnCount = 0;
//     int sampleSize = 10;
//     float distanceToWall;
//     float leftValue;
//     float leftValueData[WALL_ORIENT_READ_COUNT];
//     float rightValue;
//     float previousLeftValue;
//     float previousRightValue;
//     float rotationAngle = -30;
//     float diff = 0;
//     float totalLength = 0;
//     float samples[sampleSize];
//     float minSample;
//     long numberOfTurns = 360.0 / WALL_ORIENT_ANGLE;
//     uint32_t startTime = millis();
//     uint32_t sleepTime = millis();
//     uint32_t runTime = 10000;
//     bool ramp = false;
//     Serial.println("Pole Searching now");
//     //
//     //Determine which sensor is looking for pole.
//     if (isUpsideDown) {
//         sensor = ultrasonicRight;
//     }
//     else {
//         sensor = ultrasonicLeft;
//     }
//     // 
//     // The first few measurements of the sensor usually is very off.
//     // "Warm up" the sensor by measuring a few times.
//     while (--initIterations > 0) {
//         sensor->distanceMeasure();
//         delay(30);
//     }

//     // Rotate while finding minima, taking in both sensor values. If sum < 240 & minima, then it's either wall or ramp.
//     // If sum > 

//     // for (turnCount = 0; turnCount < numberOfTurns; turnCount++) {
//     //     drive->turnTheta(WALL_ORIENT_ANGLE);

//     //     leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);
//     //     distanceToWall = min(leftValue, distanceToWall);
//     // }

//     // do {
//     //     drive->turnTheta(WALL_ORIENT_ANGLE);
//     //     leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);
//     // } while (abs(distanceToWall - leftValue) < WALL_ORIENT_TOLERANCE);

//     // Finished turning



//     drive->poleSearchTurn(-360, ultrasonicLeft, ultrasonicRight);



//     // do {
//     //     leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);
//     //     rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT); 
//     //     drive->turnTheta(-10);
//     //     drive->reset(30);
//     //     Serial.print(leftValue);
//     //     Serial.print(" ");
//     //     Serial.print(rightValue);
//     //     Serial.println();
//     // } while (leftValue + rightValue > 150);
//     // Serial.println("FOUND");


//     // leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);

//     // minSample = leftValue;
//     // rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT);

//     // while (leftValue + rightValue > 150) {
//     //     drive->turnTheta(-7);
//     //     drive->update();

//     //     leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);
//     //     rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT);
//     //     delay(30);
//     // }

//     // // TEST: Back up to wall

//     // // Drive straight for 7s
//     // Serial.print("driving for 3s");
//     // drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
//     // drive->update();

//     // startTime = millis();
//     // do {
//     //     drive->update();
//     //     delay(30);
//     // } while (millis() - startTime < 6000);
    
//     // drive->reset(30);

//     // Serial.println("going backwards");
//     // drive->setReference(-ROBOT_SPEED_MAX / 2.0, 0.0f);
//     // drive->update();

//     // startTime = millis();
//     // do {
//     //     drive->update();
//     //     delay(30);
//     // } while (millis() - startTime < 1000);

//     // // Turn 90 deg
//     // Serial.println("Turning 90 degress");
//     // drive->turnTheta(-90);

//     // // Drive straight for 7s
//     // Serial.println("driving for 3s");

//     // drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
//     // drive->update();

//     // startTime = millis();
//     // do {
//     //     drive->update();
//     //     delay(30);
//     // } while (millis() - startTime < 6000);

//     // leftValue = readUltrasonic(ultrasonicLeft, WALL_ORIENT_READ_COUNT);
//     // rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT);

//     // if (leftValue < rightValue) {
//     //     // left sensor next to edge
//     //     tracker = ultrasonicRight;
//     //     if (rightValue < 230) {
//     //         if (rightValue < 140) {
//     //             Serial.print("Under ramp");
//     //             ramp = true;
//     //         }
//     //         else {
//     //             if (rightValue > 200) {
//     //                 Serial.print("Safe");
//     //             }
//     //             else {
//     //                 Serial.print("Over ramp");
//     //                 ramp = true;
//     //             }
//     //         }
//     //     }
//     //     else {
//     //         Serial.print("Over ramp2");
//     //         ramp = true;
//     //     }
//     // }
//     // else {
//     //     // right sensor next to edge
//     //     tracker = ultrasonicLeft;
//     //     if (leftValue < 230) {
//     //         if (leftValue < 140) {
//     //             Serial.print("Under ramp");
//     //             ramp = true;
//     //         }
//     //         else {
//     //             if (leftValue > 200) {
//     //                 Serial.print("Safe");
//     //             }
//     //             else {
//     //                 Serial.print("Over ramp");
//     //                 ramp = true;
//     //             }
//     //         }
//     //     }
//     //     else {
//     //         Serial.print("Over ramp2");
//     //         ramp = true;
//     //     }
//     // }
//     // Serial.print(" ");
//     // Serial.print(rightValue);
//     // Serial.println();
//     // delay(3000);
//     // drive->reset(30);
//     // drive->setReference(-ROBOT_SPEED_MAX / 2.0, 0.0f);
//     // drive->update();

//     // startTime = millis();
//     // do {
//     //     drive->update();
//     //     delay(30);
//     // } while (millis() - startTime < 1000);
//     // drive->reset(30);
    
//     // drive->turnTheta(180);
//     // drive->stop();
//     // drive->update();
//     // drive->reset(30);
//     // drive->update();


//     // for (int i = 0; i < numberOfSteps; i++) {
//     //     drive->setReference(ROBOT_SPEED_MAX / (numberOfSteps - i), 0.0f);
//     //     drive->update();
//     //     startTime = millis();
//     //     do {
//     //         delay(30);
//     //     } while (millis() - startTime < 500);
//     //     drive->reset(30);
//     // }

//     // drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
//     // drive->update();

//     // trackerValue = readUltrasonic(tracker, WALL_ORIENT_READ_COUNT);
//     // do {
//     //     drive->update();
//     //     previousTrackerValue = trackerValue;
//     //     trackerValue = readUltrasonic(tracker, WALL_ORIENT_READ_COUNT);
//     //     if (ramp) {
//     //         startTime = millis();
//     //         do {
//     //             drive->update();
//     //             delay(30);
//     //         } while (millis() - startTime < 2000);

//     //         ramp = false;
//     //     }
//     //     delay(30);
//     // } while (abs(previousTrackerValue - trackerValue) < WALL_ORIENT_ANGLE);
//     // drive->reset(30);
//     // rotationAngle = atan((trackerValue + 15.0f) / (ULTRASONIC_SEPARATION_DISTANCE / 2)) * 180 / M_PI;
//     // Serial.print("FOUND. Rotation angle: ");
//     // Serial.println(rotationAngle);
//     // Serial.println("Rotating now");
//     // drive->turnTheta(rotationAngle);
//     // Serial.println("Finished rotating");

    


//     // // 2. Rotate robot 360 deg till pole is found, then turn X degree to that and go straight to it
//     // rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT);

//     // Serial.println("Start searching");

//     // do {
//     //     Serial.println(readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT));
//     //     delay(400);
//     // } while (1);

//     // do {
//     //     Serial.print("Searching");
//     //     drive->turnTheta(8);
//     //     Serial.print(" Turned ");
//     //     startTime = millis();
        
//     //     previousRightValue = rightValue;
//     //     rightValue = readUltrasonic(ultrasonicRight, WALL_ORIENT_READ_COUNT);
//     //     Serial.print(previousRightValue);
//     //     Serial.print(" ");
//     //     Serial.print(rightValue);
//     //     if (millis() - startTime > DRIVE_SLEEP_TIME){
//     //         drive->update();
//     //     }
//     //     Serial.print(" ");
//     //     Serial.print(previousRightValue - rightValue);
//     //     Serial.println();
//     //     diff = previousRightValue - rightValue;
//     // } while (abs(diff) < 15.0f);


//     // rotationAngle = atan((rightValue + 15.0f) / (ULTRASONIC_SEPARATION_DISTANCE / 2)) * 180 / M_PI;
//     // Serial.print("Pole found. Rotation Angle:");
//     // Serial.print(rotationAngle);
//     // Serial.println();
//     // startTime = millis();

//     // sleepTime = DRIVE_SLEEP_TIME - (millis() - startTime);
//     // sleepTime = sleepTime > 0 ? sleepTime : 0;
//     // delay(sleepTime);
//     // // drive->turnTheta(rotationAngle);
//     // Serial.println("Finished turning");
// }
