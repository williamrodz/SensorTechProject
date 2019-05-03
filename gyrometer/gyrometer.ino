#include <MPU9250_asukiaaa.h>
MPU9250 mySensor;
float gX, gY, gZ;

void setup() {
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginGyro();
}

void loop() {
  Serial.print("test");
  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  // Do what you want
}
