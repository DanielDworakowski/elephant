#include "PID.hpp"
#include "arduino.h"

PID::PID(float p, float i, float d, float min, float max)
	: p_(p)
	, i_(i)
	, d_(d)
	, min_(min)
	, max_(max)
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
	float tDiff = (millis() - lastTime_) / 1000.0;
	// 
	// Prevent division by zero. 
	if (tDiff <= 0.0) {
		return 0;
	}
	float cmd = err * p_ + err * tDiff * i_ + (err - lastErr_) / tDiff;
	lastErr_ = err;
	return cmd;
}