#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

void setup()
{
  Serial.begin(115200);
}
 
void loop()
{
    int i;
    Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    AFMS.begin();
    Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
    myMotor->setSpeed(200);
    myMotor->run(RELEASE);
    while (1) {
        myMotor->run(FORWARD);
        for (i=0; i<255; i++) {
            myMotor->setSpeed(i);  
            delay(3);
        }
     
        for (i=255; i!=0; i--) {
            myMotor->setSpeed(i);  
            delay(3);
        }
        myMotor->run(BACKWARD);
        for (i=0; i<255; i++) {
            myMotor->setSpeed(i);  
            delay(3);
        }
     
        for (i=255; i!=0; i--) {
            myMotor->setSpeed(i);  
            delay(3);
        }

    }
}
 
