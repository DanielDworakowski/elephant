#include "VL53L0X.h"
#include <Wire.h>

typedef struct {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
} kalman_state;

kalman_state
kalman_init(double q, double r, double p, double intial_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = intial_value;

  return result;
}

void
kalman_update(kalman_state* state, double measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k) * state->p;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
}

void HandleDistInterrupt()
{
    // Serial.println("Got interrupt!!!");
    // Set interrupt satus for reading later. 
}

// TODO: implement new sample through GPIO1 interrupt (if continuous).
void loop()
{
    VL53L0X sensor;
    sensor.init();
    sensor.setTimeout(500);
    // 
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // 
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //
    // Set the timing budget for higher accuracy. 
    sensor.setMeasurementTimingBudget(70000); // 70000 works well
    // 
    // Set continuous mode, therefore need to setup the interrupts.
    sensor.startContinuous();

    kalman_state state = kalman_init(1, 2, 1, 0);

    attachInterrupt(digitalPinToInterrupt(2), HandleDistInterrupt, RISING);

    float meas = 0;
    while(1) {
        meas = sensor.readRangeContinuousMillimeters();
        kalman_update(&state, meas);
        Serial.print(state.x);
        Serial.print(" ");
        Serial.print(meas);
        // Serial.print(" [mm]");
        if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        Serial.println();
    }
}
 
