#include "PinDefines.h"
#include "VL53L0X.h"
#include <Wire.h>

void setup()
{
    Serial.begin(115200);
    Wire.begin();
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
    // sensor.setMeasurementTimingBudget(20000);


    while(1) {
        Serial.print(sensor.readRangeSingleMillimeters());
        Serial.print(" [mm]");
        if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        Serial.println();
    }
}
 
