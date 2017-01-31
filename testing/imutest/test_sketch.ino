// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
#include "IMU_Wrapper.hpp"
#define IMU_INTERRUPT_PIN 18

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);
}

void loop() {

    // Lets make our own loop...
    IMU imu(IMU_INTERRUPT_PIN);

    while (1) {
        imu.read();
        delay(100);
    }
}

