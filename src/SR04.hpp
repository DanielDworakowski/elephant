#include <Arduino.h>

class SR04
{
    public:
        SR04(int trig_pin, int echo_pin);
        void measureDistance(void);
        long microsecondsToCentimeters(void);
        long microsecondsToInches(void);
        unsigned long getTimestamp(void);
    
    private:
        int _trig_pin; // Pin number of the trigger pin.
        int _echo_pin; // Pin number of the echo pin.
    
        unsigned long _pulse_width = 0;
        unsigned long _timestamp = 0;
};


