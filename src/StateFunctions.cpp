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
    //
    // Acceleration by the number of steps equally. 
    for (step = 0; step < numSteps; ++step) {
        startTime = millis();
        drive->setReference(-ROBOT_SPEED_MAX * (step / numSteps), 0.0f);
        while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME / numSteps) {
            drive->update();
            meas = prox->readRangeContinuousMillimeters();
            if (meas < WALL_JUMP_DIST && ((millis() - minTriggerStart) > minTriggerTime)) {
                break;
            }
        }
    }
    // 
    // We are done accelerating make sure the drive is going in steady state.
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

int StateFunctions::orientForward(Drive *drive, Ultrasonic* ultrasonicL, Ultrasonic* ultrasonicR)
{
    //
    // Check if one side is too close to rotate and treat it as the wall?
    // ^ Would likely need a proper set distance for this to avoid dying.  
    // make assumption about minimal wall distance and rotate until there is a tolerance
    // follow the wall as required. 
}

int StateFunctions::checkUpsideDown(Drive *drive, VL53L0X *prox)
{
    drive->setUpsideDown(prox->isUpsideDown());
    drive->reset(30);
    return 0;
}

int locateDriveHelper(Drive *drive, float curDist, float setDist)
{
    // static PID distPID(LOCATE_P, LOCATE_I, LOCATE_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    // float cmd = 0;
    // cmd = distPID.getCmd(setDist, curDist);
    // drive->setOmega(cmd);
    drive->update();
    return 0;
}

/*
 *  Parameters: pointer to drive motor interface instance, pointer to left and right ultrasonic interface.
 *  Continues driving until either of the ultrasonics detect a large difference in measurement.
 *  Then the robot turns in that direction.
 */
int StateFunctions::locateDest(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight, VL53L0X *prox)
{   
    Ultrasonic *poleSensor;
    Ultrasonic *wallSensor;
    bool isUpsideDown;
    long poleData[3];
    long poleCurrentData;
    long poleLastData;
    long wallData[3];
    long wallCurrentData;
    long wallLastData;
    int initIterations = 10;
    uint32_t startTime = millis();
    int32_t sleepTime = 0;
    float initialLDist = 0;
    int confirmationCheckCount = POLE_CONFIRMATION_CHECK_COUNT;
    int confirmationCheckPassed = 0;

    dprintln("//// State - enter locateDest.");

    //
    // Check if the robot is upside down. 
    isUpsideDown = prox->isUpsideDown();

    //
    //Determine which sensor is looking for pole.
    if (isUpsideDown) {
        poleSensor = ultrasonicLeft;
        wallSensor = ultrasonicRight;
    }
    else {
        poleSensor = ultrasonicRight;
        wallSensor = ultrasonicLeft;
    }

    // 
    // The first few measurements of the sensor usually is very off.
    // "Warm up" the sensor by measuring a few times.
    while (--initIterations > 3) {
        wallSensor->distanceMeasure();
        poleSensor->distanceMeasure();

        delay(30);
    }

    // 
    // Take initial measurements to fill data buffer.
    while (--initIterations >= 0) {
        poleSensor->distanceMeasure();
        poleData[2 - initIterations] = poleSensor->microsecondsToCentimeters();

        wallSensor->distanceMeasure();
        wallData[2 - initIterations] = wallSensor->microsecondsToCentimeters();
    }

    // 
    // Compute median.
    poleCurrentData = max(min(poleData[0], poleData[1]), min(max(poleData[0], poleData[1]), poleData[2]));
    wallCurrentData = max(min(wallData[0], wallData[1]), min(max(wallData[0], wallData[1]), wallData[2]));
    initialLDist = wallCurrentData;
    dprint("Pole: ");
    dprint(poleCurrentData);
    dprint(" Wall: ");
    dprintln(wallCurrentData);
    
    //
    // This outer while loop runs until the pole has been confirmed to be found.
    // If the confirmation check fails, it will loop back to going forward and searching (inner loop).
    do {
        // 
        // Start moving.
        drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
        drive->update();
        //
        // This inner loop drives and takes measurements until something that appears to be the pole is detected.
        do {
            startTime = millis();
            // 
            // Measure data and shift down data buffer.
            poleSensor->distanceMeasure();
            poleData[0] = poleData[1];
            poleData[1] = poleData[2];
            poleData[2] = poleSensor->microsecondsToCentimeters();
            // 
            // Save previous data and compute new data from median.
            poleLastData = poleCurrentData;
            poleCurrentData = max(min(poleData[0], poleData[1]), min(max(poleData[0], poleData[1]), poleData[2]));
            // 
            // Update the drive if needed.
            if ((millis() - startTime) > DRIVE_SLEEP_TIME){
                drive->update();
            }
            // 
            // Measure and evaluate the right sensor.
            wallSensor->distanceMeasure();
            wallData[0] = wallData[1];
            wallData[1] = wallData[2];
            wallData[2] = wallSensor->microsecondsToCentimeters();
            // 
            // Save data.
            wallLastData = wallCurrentData;
            wallCurrentData = max(min(wallData[0], wallData[1]), min(max(wallData[0], wallData[1]), wallData[2]));
            locateDriveHelper(drive, wallCurrentData, initialLDist); // Drive->update() is called in here.
            // 
            // Left right ultrasonics. 
            dprint("Pole: ");
            dprint(poleCurrentData);
            dprint(" Wall: ");
            dprintln(wallCurrentData);
            // 
            // Calculate how much to sleep to keep controller stable.
            sleepTime = DRIVE_SLEEP_TIME - (millis() - startTime);
            sleepTime = sleepTime > 0 ? sleepTime : 0;
            delay(sleepTime);
        } while (abs(poleCurrentData - poleLastData) < POLE_DELTA_TOLERANCE || poleCurrentData > POLE_NOISE_CEIL);
        //
        // Do confirmation check on the pole measurement.
        drive->stop();
        confirmationCheckCount = POLE_CONFIRMATION_CHECK_COUNT;
        confirmationCheckPassed = 0;
        while (confirmationCheckCount > 0) {
            poleSensor->distanceMeasure();
            // 
            // Check if the measurement was not a mistake through a tolerance. 
            if (abs(poleSensor->microsecondsToCentimeters() - poleCurrentData) <= POLE_CONFIRMATION_TOLERANCE) {
                confirmationCheckPassed++;
            }
            confirmationCheckCount--;
            delay(30);
        }
    } while (confirmationCheckPassed < POLE_CONFIRMATION_CHECK_COUNT - POLE_CONFIRMATION_CHECK_FAIL_TOLERANCE);
    // 
    // May need to implement something to check if crashed. Also may need better method of handling false negative, maybe backing up.
    dprintln("Pole found.");
    drive->turnTheta(-90);
    dprintln("//// State - exit locateDest.");
    return 0;
}

int StateFunctions::driveToDest(Drive *drive, IMU *imu)
{   
    while (imu->read() != 0) {};
    float initZ = imu->getGlobalZ();
    float newZ;
    // 
    // Start moving the robot. 
    drive->setReference(ROBOT_SPEED_MAX / 2.0, 0.0f);
    drive->update();
    delay(20);
    // 
    // Continue until a bump is detected. 
    do {
        imu->read();
        newZ = imu->getGlobalZ();
        drive->update();
        dprint("initZ: ");
        dprint(initZ);
        dprint(" newZ; ");
        dprintln(newZ);
        delay(30);
    } while (1);
    drive->stop();
    return 0;
}
