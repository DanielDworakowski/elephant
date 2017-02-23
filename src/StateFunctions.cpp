#include "StateFunctions.hpp"

int StateFunctions::approachState(Drive *drive, VL53L0X* prox) 
{
    PID acc(ACC_P, ACC_I, ACC_D, SPEED_MAX, SPEED_MIN);
    float meas = 0, cmd = 0;
    // 
    // Monitor and control the speed using the PID and 
    do {
        do {
            meas = prox->readRangeContinuousMillimeters();
            cmd = -1.0f * acc.getCmd(WALL_SET_DIST, meas);
            drive->setReference(cmd, 0.0f);
            drive->update();
            Serial.print(meas);
            Serial.print("\t");
            Serial.println(cmd);
        } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
        // 
        // Stop and ensure no drift. 
        drive->stop();
        delay(10);
    } while (abs(meas - WALL_SET_DIST) > WALL_DIST_TOL);
    return 0;
}

int waitForStartButton()
{
    while (!digitalRead(START_BUTTON_PIN)) {
        delay(10);
    }
    delay(1000);
    return 0;
}

int getOffPlateform(Drive *drive)
{
    float curTime = millis();
    while (millis()-curTime < 1000) {
        drive->setReference(SPEED_MAX, 0.0f);
        drive->update();
    }
    return 0;
    //
    // Does not have drive->stop to ensure a fast transition discussion needed (DN)
}
