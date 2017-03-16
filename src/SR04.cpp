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
#include "SR04.hpp"

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;
const float CONVERSION_CONSTANT_CM = 1 / 58.0;
const float CONVERSION_CONSTANT_INCH = 1 / 148.0;

SR04::SR04(int trig_pin, int echo_pin)
{
    _trig_pin = trig_pin;
    _echo_pin = echo_pin;

    // Initialize trigger pin.
    pinMode(_trig_pin, OUTPUT);
    digitalWrite(_trig_pin, LOW);
}

void SR04::DistanceMeasure(void) 
{
    unsigned long t1;
    unsigned long t2;

    // Hold the trigger pin high for at least 10 us
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);

    // Wait for pulse on echo pin
    while ( digitalRead(_echo_pin) == 0 );

    // Measure how long the echo pin was held high (pulse width)
    // Note: the micros() counter will overflow after ~70 min
    t1 = micros();
    while ( digitalRead(_echo_pin) == 1 );
    t2 = micros();

    _pulse_width = t2 - t1;
    _timestamp = millis();
}

long SR04::microsecondsToCentimeters(void)
{
    return (long)(_pulse_width > MAX_DIST ? 400 : CONVERSION_CONSTANT_CM * _pulse_width);
}


long SR04::microsecondsToInches(void) 
{
    return (long)(_pulse_width > MAX_DIST ? 400 : CONVERSION_CONSTANT_INCH * _pulse_width);
}

unsigned long SR04::getTimestamp(void)
{
    return _timestamp;
}