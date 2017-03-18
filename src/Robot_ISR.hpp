#include "PinDefines.h"

#ifndef __ISR_H__
#define __ISR_H__

volatile int32_t gLeftEncoderTicks = 0;
volatile int32_t gRightEncoderTicks = 0;

#define LeftEncoderIsReversed
// #define RightEncoderIsReversed

void HandleLeftMotorInterruptA()
{
#ifdef LeftEncoderIsReversed
    gLeftEncoderTicks -= digitalReadFast(PIN::leftEncoderPinB) ? -1 : +1;
#else
    gLeftEncoderTicks += digitalReadFast(PIN::leftEncoderPinB) ? -1 : +1;
#endif
}
 
void HandleRightMotorInterruptA()
{
#ifdef RightEncoderIsReversed
    gRightEncoderTicks -= digitalReadFast(PIN::rightEncoderPinB) ? -1 : +1;
#else
    gRightEncoderTicks += digitalReadFast(PIN::rightEncoderPinB) ? -1 : +1;
#endif
}

/*
 * Setup all pins. */
void setupPins() 
{
    // 
    // Encoders.
    pinMode(PIN::leftEncoderPinB, INPUT);
    digitalWrite(PIN::leftEncoderPinB, LOW);
    pinMode(PIN::rightEncoderPinB, INPUT);
    digitalWrite(PIN::rightEncoderPinB, LOW);
    pinMode(PIN::leftEncoderInterruptPin, INPUT);
    pinMode(PIN::rightEncoderInterruptPin, INPUT);
    // 
    // Attach all interrupts. 
    attachInterrupt(digitalPinToInterrupt(PIN::leftEncoderInterruptPin), HandleLeftMotorInterruptA, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN::rightEncoderInterruptPin), HandleRightMotorInterruptA, RISING);
    // 
    // Start Button.
    pinMode(PIN::startButtonPin, INPUT);
    // 
    // Alignment button.
    pinMode(PIN::alignButtonPin, INPUT_PULLUP);
}

#endif