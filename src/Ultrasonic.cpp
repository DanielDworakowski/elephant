/**
 * HC-SR04 Demo
 * Demonstration of the HC-SR04 Ultrasonic Sensor
 * Date: August 3, 2016
 * 
 * Description:
 *  Connect the ultrasonic sensor to the Arduino as per the
 *  hardware connections below. Run the sketch and open a serial
 *  monitor. The distance read from the sensor will be displayed
 *  in centimeters and inches.
 * 
 * Hardware Connections:
 *  Arduino | HC-SR04 
 *  -------------------
 *    5V    |   VCC     
 *    7     |   Trig     
 *    8     |   Echo     
 *    GND   |   GND
 *  
 * License:
 *  Public Domain
 */

#include "Ultrasonic.hpp"

#define DEBUG
#ifdef DEBUG

#define dprint(x) do { Serial.print(x); } while (0)
#define dprintln(x) do { Serial.println(x); } while (0)

#else

#define dprint(x) 
#define dprintln(x) 

#endif 

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;
const float CONVERSION_CONSTANT_CM = 1 / 58.2;
const float CONVERSION_CONSTANT_INCH = 1 / 148.0;

Ultrasonic::Ultrasonic(int trig_pin, int echo_pin)
{
    _trig_pin = trig_pin;
    _echo_pin = echo_pin;

    pinMode(_trig_pin, OUTPUT);
    pinMode(_echo_pin, INPUT);

    dprintln("//// Hardware - SR04 Ultrasonic Sensor initialized.");
}

void Ultrasonic::distanceMeasure(void) 
{
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);
      
    pinMode(_echo_pin, INPUT);
    _pulse_width = pulseIn(_echo_pin, HIGH);
    _timestamp = millis();

    dprint("SR04 - Measured distance ");
    dprint(_pulse_width > MAX_DIST ? 400 : CONVERSION_CONSTANT_CM * _pulse_width);
    dprintln(" cm.");
}

long Ultrasonic::microsecondsToCentimeters(void)
{
    return (long)(_pulse_width > MAX_DIST ? 400 : CONVERSION_CONSTANT_CM * _pulse_width);
}

long Ultrasonic::microsecondsToInches(void) 
{
    return (long)(_pulse_width > MAX_DIST ? 400 : CONVERSION_CONSTANT_INCH * _pulse_width);
}

unsigned long Ultrasonic::getTimestamp(void)
{
    return _timestamp;
}