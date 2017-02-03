#include "PID.hpp"
#include "arduino.h"

PID::PID(float p, float i, float d)
    : p_(p)
    , i_(i)
    , d_(d)
    , lastErr_(0)
{
    lastTime_ = millis();
}

PID::~PID()
{
}

float PID::getError(float ref, float meas)
{
    float err = ref - meas;
    float curTime = millis();
    float tDiff = (curTime - lastTime_) / 1000.0;
    // 
    // Prevent division by zero. 
    if (tDiff <= 0.0) {
        return 0;
    }
    float cmd = err * p_ + err * tDiff * i_ + d_ * (err - lastErr_) / tDiff;
    lastErr_ = err;
    lastTime_ = curTime;
    return cmd;
}