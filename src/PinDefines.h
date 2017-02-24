#ifndef __PIN_DEFINITIONS_H__
#define __PIN_DEFINITIONS_H__

enum PIN: byte
{
	// 
	// Quadrature encoders.
	leftEncoderInterruptPin = 19, // A side of encoder
	leftEncoderPinB = 17,
	rightEncoderInterruptPin =  5, // A side of encoder
	rightEncoderPinB =  24,
	// 
	// IMU interrupt pin.
	imuInterruptPin = 18,
	// 
	// Proximity sensor interrupt pins.
	tofStage1InterruptPin = 0,
	tofStage2InterruptPin = 0,
	//
	// Start button pin
	startButtonPin = 31,
};

#endif