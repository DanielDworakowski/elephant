#include "StateFunctions.hpp"
#include "Kalman-1D.hpp"

#define DEBUG true
#define dprint(x) do { if (DEBUG) Serial.print(x); } while (0)
#define dprintln(x) do { if (DEBUG) Serial.println(x); } while (0)

/* 
 * Parameters: none
 * Waits for button press, then a certain amount of time before continuing.
 */
int StateFunctions::waitForStartButton(void)
{
    dprintln("//// State - enter waitForStartButton.");
    while (!digitalRead(PIN::startButtonPin)) {
        if (!digitalRead(PIN::alignButtonPin)) {
            // Code to align gear here.
        }
    }

    dprintln("//// State - exit waitForStartButton.");
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

    // Give 2 seconds for the IMU to stablize, as well as to get out of the way.
    delay(IMU_STABLIZE_TIME);
    
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

/*
 *  Parameters: pointer to drive motor interface instance.
 *  Drives for a set amount of time, defined by DRIVE_OFF_PLATFORM_TIME.
 */
int StateFunctions::getOffPlatform(Drive *drive)
{
    float startTime = millis();

    dprintln("//// State - enter getOffPlatform.");

    while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME) {
        drive->setReference(1.0 * ROBOT_SPEED_MAX, 0.0f);
        drive->update();
        delay(30);
    }

    dprintln("//// State - exit getOffPlatform.");
    return 0;
}

/*
 *  Parameters: pointer to drive motor interface instance, and pointer to ToF interface instance.
 *  Drive forward using PID to ensure direction, until a certain distance to the wall is reported by ToF sensor.
 */
int StateFunctions::approach(Drive *drive, VL53L0X* prox) 
{
    PID acc(ACC_P, ACC_I, ACC_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    float meas = 0, cmd = 0;

    dprintln("//// State - enter approach.");
    
    // Monitor and control the speed using the PID and 
    do {
        meas = prox->readRangeContinuousMillimeters();
        
        //cmd = acc.getCmd(WALL_SET_DIST, meas);
        //drive->setReference(-1.0 * cmd, 0.0f);
        //drive->update();
        delay(30);
    } while (meas > WALL_JUMP_DIST);

    dprintln("//// State - exit approach.");
    return 0;
}

/*
 *  Parameters: pointer to instance of jump motor interface instance, and pointer to instance of IMU interface instance.
 *  Runs the jump motor at max speed for a certain amount of time to unlatch the mechanism.
 */
int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, IMU *imu)
{
    dprintln("//// State - enter jump.");

    // Run the motor forwards until acceleration is detected.
    jumpMotor->run(BACKWARD);
    jumpMotor->setSpeed(255);
    delay(1000);
    // 
    // Stop the motor.
    jumpMotor->setSpeed(0);

    dprintln("//// State - exit jump.");
    return 0;
}

/*
 *  Parameters: pointer to instance of drive motor interface instance, and pointer to instance of IMU interface instance.
 *  Uses the drive motors and wheel rotation to stablize the robot while in flight.
 *  NOTE: Probably going to be unused as it is not necessary and also it relies on the IMU.
 */
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

/*
 *  Parameters: pointer to drive motor interface instance, pointer to IMU interface instance, yaw value to match.
 *  Turns the robot on the spot to orientate itself to parallel to the wall by matching the reference yaw value.
 *  NOTE: Potentially not needed if robot jumps well. Also, IMU needs tuning.
 */
int StateFunctions::orientForward(Drive *drive, IMU *imu, float refYaw)
{
    float cmd = 0.0;
    float curYaw = 0.0;

    dprintln("//// State - enter orientForward.");

    imu->read();
    curYaw = imu->getYaw();
    
    PID pid(ORIENT_P, ORIENT_I, ORIENT_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);
    
    while (abs(curYaw - refYaw) > YAW_TOL) {
        imu->read();
        curYaw = imu->getYaw();
        
        cmd = pid.getCmd(refYaw, curYaw);
        drive->setReference(0, imu->isUpsideDown() ? -cmd : cmd);
        drive->update();

        dprint("IMU - reference yaw ");
        dprint(curYaw);
        dprint(" and current yaw ");
        dprintln(refYaw);

        delay(10);
    } 

    drive->stop();

    dprintln("//// State - exit orientForward.");
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

    dprintln("//// State - enter locateDest.");

    //drive->setReference(-1.0 * ROBOT_SPEED_MAX / 8.0, 0.0f);
    //drive->update();

    // The first few measurements of the sensor usually is very off.
    // "Warm up" the sensor by measuring a few times.
    while (--initIterations > 3) {
        ultrasonicLeft->distanceMeasure();
        ultrasonicRight->distanceMeasure();

        delay(30);
    }

    // Take initial measurements to fill data buffer.
    while (--initIterations >= 0) {
        ultrasonicLeft->distanceMeasure();
        leftData[2 - initIterations] = ultrasonicLeft->microsecondsToCentimeters();

        ultrasonicRight->distanceMeasure();
        rightData[2 - initIterations] = ultrasonicRight->microsecondsToCentimeters();
    }

    // Compute median.
    leftCurrentData = max(min(leftData[0], leftData[1]), min(max(leftData[0], leftData[1]), leftData[2]));
    rightCurrentData = max(min(rightData[0], rightData[1]), min(max(rightData[0], rightData[1]), rightData[2]));
    
    do {
        // Measure then evaluate delta.
        ultrasonicLeft->distanceMeasure();
        leftData[0] = leftData[1];
        leftData[1] = leftData[2];
        leftData[2] = ultrasonicLeft->microsecondsToCentimeters();

        leftLastData = leftCurrentData;
        leftCurrentData = max(min(leftData[0], leftData[1]), min(max(leftData[0], leftData[1]), leftData[2]));
        
        ultrasonicRight->distanceMeasure();
        rightData[0] = rightData[1];
        rightData[1] = rightData[2];
        rightData[2] = ultrasonicRight->microsecondsToCentimeters();

        rightLastData = rightCurrentData;
        rightCurrentData = max(min(rightData[0], rightData[1]), min(max(rightData[0], rightData[1]), rightData[2]));

        //drive->update();
        delay(30);
    //} while (1);
    } while (abs(leftCurrentData - leftLastData) < ULTRASONIC_DELTA_TOLERANCE && abs(rightCurrentData - rightLastData) < ULTRASONIC_DELTA_TOLERANCE);

    dprint("Pole found ");
    //drive->stop();
    
    if (abs(leftCurrentData - leftLastData) > ULTRASONIC_DELTA_TOLERANCE) {
        dprintln("on left side.");
        //drive->turnLeft();
    }
    else {
        dprintln("on right side.");
        //drive->turnRight();
    }

    dprintln("//// State - exit locateDest.");
    return 0;
}

/*
 *  Parameters: pointer to drive motor interface instance, pointer to IMU interface instance.
 *  Drive forward until a certain upward acceleration is detected by the IMU.
 */
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
