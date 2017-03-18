#ifndef __PIN_DEFINITIONS_H__
#define __PIN_DEFINITIONS_H__

#include <Arduino.h>

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
    // SR04 Ultrasonic Sensor (left) --- NEEDS TO BE DEFINED
    leftUltrasonicTrigPin = 7,
    leftUltrasonicEchoPin = 6,
    //
    // RB90 Ultrasonic Sensor (right) --- NEEDS TO BE DEFINED
    rightUltrasonicTrigPin = 4,
    rightUltrasonicEchoPin = 3,
    //
    // Align button.
    alignButtonPin = 53
};

#endif