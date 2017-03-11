#ifndef __PIN_DEFINITIONS_H__
#define __PIN_DEFINITIONS_H__

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

#endif