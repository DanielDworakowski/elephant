#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

enum PIN: byte
{
  // 
  // Quadrature encoders.
  leftEncoderInterruptPin = 19, // A side of encoder
  leftEncoderPinB = 17,
  rightEncoderInterruptPin =  18, // A side of encoder
  rightEncoderPinB =  16,
  // 
  // IMU interrupt pin.
  imuInterruptPin = 2,
  // 
  // Proximity sensor interrupt pins.
  tofStage1InterruptPin = 0,
  tofStage2InterruptPin = 0,
  //
  // Start button pin
  startButtonPin = 8,
    //
    // Ultrasonic sensor
    leftUltrasonicPin = 111,
    rightUltrasonicPin = 222,
};

int waitForStartButton()
{
    while (!digitalRead(PIN::startButtonPin)) {
        Serial.println("Waiting for button press");
    }
    Serial.println("Button was pressed!");
    return 0;
}

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);
    // 
    // Start Button
    pinMode(PIN::startButtonPin, INPUT);
}

void loop()
{ 
    // 
    // Define objects.
    Adafruit_MotorShield motorShield;
    // 
    // Begin sensing.
    motorShield.begin();
    // 
    // Begin the state machine.
    while (1) {
        waitForStartButton();
        motorShield.getMotor(3)->run(FORWARD);
        motorShield.getMotor(3)->setSpeed(255.0f / 4.0f);
        delay(50);
        motorShield.getMotor(3)->setSpeed(0);
    }
}
 
