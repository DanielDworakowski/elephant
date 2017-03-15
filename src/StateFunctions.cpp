#include "StateFunctions.hpp"

/* 
 * Parameters: none
 * Waits for button press, then a certain amount of time before continuing.
 */
int StateFunctions::waitForStartButton()
{
    while (!digitalRead(PIN::startButtonPin)) {}
    delay(3000);
    return 0;
}

/* 
 *  Parameters: pointer to IMU interface instance, address of yaw storage variable.
 *  Takes a number of samples of the current yaw and saves the average to the pointer.
 */
int StateFunctions::sampleYaw(IMU *imu, float &yaw)
{
    uint16_t cnt; 
    const uint16_t avgNum = 15; // Number of samples to average.

    dprintln("Start averaging for yaw.");

    // Average the current yaw.
    for (cnt = 0; cnt < avgNum; ++cnt) {
        // IMU read could fail, do timeout and retry.
        if (imu->read() < 0) {
            --cnt;
            continue;
        }

        yaw += imu->getYaw();
        dprintln(imu->getYaw());
        delay(40);
    }

    yaw /= avgNum;
    dprint("Done averaging. Average yaw: ");
    dprintln(yaw);
    return 0;
}

/*
 *  Parameters: pointer to drive motor interface instance.
 *  Drives for a set amount of time, defined by DRIVE_OFF_PLATFORM_TIME.
 */
int StateFunctions::getOffPlatform(Drive *drive)
{
    float startTime = millis();
    while (millis() - startTime < DRIVE_OFF_PLATFORM_TIME) {
        drive->setReference(-1.0 * ROBOT_SPEED_MAX, 0.0f);
        drive->update();
        delay(20);
    }
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
    
    // Monitor and control the speed using the PID and ToF measured distance.
    do {
        do {
            meas = prox->readRangeContinuousMillimeters();
            cmd = acc.getCmd(WALL_SET_DIST, meas);
            drive->setReference(cmd, 0.0f);
            drive->update();
            delay(30);
        } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
        
        // Stop and ensure no drift. 
        drive->stop();
        delay(10);
    } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
    return 0;
}

/*
 *  Parameters: pointer to instance of jump motor interface instance, and pointer to instance of IMU interface instance.
 *  Runs the jump motor at max speed for a certain amount of time to unlatch the mechanism.
 */
int StateFunctions::jump(Adafruit_DCMotor *jumpMotor, IMU *imu)
{
    float startTime = millis();

    // Run the motor forwards until acceleration is detected.
    jumpMotor->run(BACKWARD);
    jumpMotor->setSpeed(255);
    
    // Run the motor for a certain amount of time to unlatch mechanism.
    while (millis() - startTime < JUMP_MOTOR_ENGAGE_TIME) {}

    // Stop the motor.
    jumpMotor->setSpeed(0);
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
    } while (imu->getGlobalZ() < JUMP_THRESHOLD); // This condition will cut out at the apex of the jump, need refactoring.
    
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
    float curYes = 0.0;
    PID pid(ORIENT_P, ORIENT_I, ORIENT_D, ROBOT_SPEED_MAX, ROBOT_SPEED_MIN);

    imu->read();
    curYaw = imu->getYaw();

    while (abs(curYaw - refYaw) > YAW_TOL) {
        imu->read();
        curYaw = imu->getYaw();

        cmd = pid.getCmd(refYaw, curYaw);
        drive->setReference(0, imu->isUpsideDown() ? -cmd : cmd);
        drive->update();
        delay(10);

        dprint(curYaw);
        dprint('\t');
        dprint(refYaw);
        dprint('\t');
        dprint(imu->getYaw());
        dprint('\t');
        dprint(imu->getPitch());
        dprint('\t');
        dprint(imu->getRoll());
        dprintln('\t');
    } 

    drive->stop();
    return 0;
}

/*
 *  Work in progress.
 */
int StateFunctions::locateDest(Drive *drive, Ultrasonic *ultrasonicLeft, Ultrasonic *ultrasonicRight)
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
    delay(30);

    if (leftSensor < 400) {
        drive->turnLeft();
    }
    else {
        drive->turnRight();
    }

    return 0;
}

/*
 *  Work in progress.
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
