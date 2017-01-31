#include "NewPing.h"
#include <Wire.h>

typedef struct {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
} kalman_state;

kalman_state kalman_init(double q, double r, double p, double intial_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = intial_value;

  return result;
}

void kalman_update(kalman_state* state, double measurement)
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

void loop()
{
    NewPing sonar(3,2,300);
    kalman_state state = kalman_init(1, 2, 1, 0);

    while (1) {
        float meas = sonar.ping_cm() * 10;
        kalman_update(&state, meas);
        Serial.print(meas);
        Serial.print(" ");
        Serial.print(state.x);
        Serial.println();
        delay(30);
    } 
}
 
