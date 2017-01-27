#include <Wire.h>
#include "digitalWriteFast.h"
#include "Robot_ISR.hpp"
#include "PID.hpp"
#include "PinDefines.h"
#include "Kalman-1D.hpp"

void setup()
{
    Serial.begin(115200);
    Wire.begin();
}

void loop()
{
}
 
