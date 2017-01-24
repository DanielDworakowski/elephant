#ifndef __ISR_H__
#define __ISR_H__

#include "PinDefines.h"

#define LeftEncoderIsReversed

volatile long gLeftEncoderTicks;
volatile long gRightEncoderTicks;

void HandleLeftMotorInterruptA()
{
  #ifdef LeftEncoderIsReversed
    gLeftEncoderTicks -= digitalReadFast(LEFT_ENCODER_PIN_B) ? -1 : +1;
  #else
    gLeftEncoderTicks += digitalReadFast(LEFT_ENCODER_PIN_B) ? -1 : +1;
  #endif
}
 
void HandleRightMotorInterruptA()
{
  #ifdef RightEncoderIsReversed
    gRightEncoderTicks -= digitalReadFast(RIGHT_ENCODER_PIN_B) ? -1 : +1;
  #else
    gRightEncoderTicks += digitalReadFast(RIGHT_ENCODER_PIN_B) ? -1 : +1;
  #endif
}

/*
 * Setup all pins. */
void setupPins() 
{
    /*
     * Encoders */
    pinMode(LEFT_ENCODER_PIN_B, INPUT);
    digitalWrite(LEFT_ENCODER_PIN_B, LOW);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT);
    digitalWrite(RIGHT_ENCODER_PIN_B, LOW);
    pinMode(LEFT_ENCODER_INTERRUPT_PIN, INPUT);
    pinMode(RIGHT_ENCODER_INTERRUPT_PIN, INPUT);
    /*
     * Attach all interrupts. */
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_INTERRUPT_PIN), HandleLeftMotorInterruptA, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_INTERRUPT_PIN), HandleRightMotorInterruptA, RISING);
}

#endif