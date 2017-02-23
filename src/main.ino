#include <Wire.h>
#include "digitalWriteFast.h"
#include "PinDefines.h"
#include "VL53L0X.h"
#include "Robot_ISR.hpp"
#include "Kalman-1D.hpp"
#include "IMU_Wrapper.hpp"
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "Drive.hpp"
#include "StateFunctions.hpp"

void setupProximity(VL53L0X &sensor) 
{
    sensor.init();
    sensor.setTimeout(500);
    // 
    // Setup sensor for long range.
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //
    // Set the timing budget for higher accuracy. 
    sensor.setMeasurementTimingBudget(70000); // 70000 works well 
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    setupPins();
}

void loop()
{ 
    // 
    // Define objects.
    VL53L0X prox;
    Adafruit_MotorShield motorShield;// = Adafruit_MotorShield();
    IMU imu(IMU_INTERRUPT_PIN);
    Drive drive(&gRightEncoderTicks, &gLeftEncoderTicks, motorShield.getMotor(2), motorShield.getMotor(1));
    drive.setReference(1,0);
    // 
    // Begin sensing.
    setupProximity(prox);
    prox.startContinuous();
    motorShield.begin();

    while (1) {
        //
        // Print data. 
        // Serial.println("Encoder");
        // Serial.print(gLeftEncoderTicks);
        // Serial.print("\t");
        // Serial.print(gRightEncoderTicks);
        // Serial.print("\n");
        drive.update();
        delay(30);

        // // 
        // // Proximity.
        // meas = prox.readRangeContinuousMillimeters();
        // if (prox.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        // Serial.println("Proximity");
        // Serial.println(meas);
        // // 
        // // IMU.
        // Serial.println("IMU");
        // imu.read();
    }
}
 
