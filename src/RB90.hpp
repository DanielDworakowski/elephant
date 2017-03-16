#include "Arduino.h"

class RB90
{
    public:
        RB90(int pin);
        void DistanceMeasure(void);
        long microsecondsToCentimeters(void);
        long microsecondsToInches(void);
        unsigned long getTimestamp(void);
    
    private:
        int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        long _duration;// the Pulse time received;
        unsigned long _timestamp;
};
