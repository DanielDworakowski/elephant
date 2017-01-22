#include "digitalWriteFast.h"
#include "ISR.h"
#include "PinDefines.h"

extern volatile long gLeftEncoderTicks;
extern volatile long gRightEncoderTicks;

void setup()
{
  Serial.begin(115200);
  setupPins();
}
 
void loop()
{
  Serial.print(gLeftEncoderTicks);
  Serial.print("\t");
  Serial.print(gRightEncoderTicks);
  Serial.print("\n");
 
  delay(20);
}
 
