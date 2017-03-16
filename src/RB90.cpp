/***************************************************************************/
//    Function: Measure the distance to obstacles in front and print the distance
//              value to the serial terminal.The measured distance is from
//              the range 0 to 400cm(157 inches).
//    Hardware: Ultrasonic Range sensor
//    Arduino IDE: Arduino-1.0
//    Author:     LG
//    Date:      Jan 17,2013
//    Version: v1.0 modified by FrankieChu
//    by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
/*****************************************************************************/
#include "RB90.hpp"

RB90::RB90(int pin)
{
    _pin = pin;
}

/*Begin the detection and get the pulse back signal*/
void RB90::distanceMeasure(void)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_pin,LOW);
    pinMode(_pin,INPUT);
    _duration = pulseIn(_pin,HIGH);
    _timestamp = millis();
}

/*The measured distance from the range 0 to 400 Centimeters*/
long RB90::microsecondsToCentimeters(void)
{
    return _duration/29/2;
}

/*The measured distance from the range 0 to 157 Inches*/
long RB90::microsecondsToInches(void)
{
    return _duration/74/2;
}

unsigned long RB90::getTimestamp(void)
{
    return _timestamp;
}
