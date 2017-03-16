#include <Arduino.h>

#ifndef __SR04_H__
#define __SR04_H__

class SR04
{
    public:
        SR04(int trig_pin, int echo_pin);
        void DistanceMeasure(void);
        long microsecondsToCentimeters(void);
        long microsecondsToInches(void);
        unsigned long getTimestamp(void);
    
    private:
        int _trig_pin; // Pin number of the trigger pin.
        int _echo_pin; // Pin number of the echo pin.
    
        unsigned long _pulse_width = 0;
        unsigned long _timestamp = 0;
};

#endif 