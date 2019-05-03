#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN A4
#define SCL_PIN A5
#endif

MPU9250 mySensor;

uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float force_x, force_y, force_z;
float pos_x, pos_y, pos_z;
float secondsSoFar;
float massOfBike;

void setup() {
  massOfBike = 12.7; //in kg
  while(!Serial);
  Serial.begin(115200);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
#else
  Wire.begin();
#endif

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  sensorId = mySensor.readId();
}

void loop() {
  Serial.println("sensorId: " + String(sensorId));

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
  aSqrt = mySensor.accelSqrt();
  Serial.println("accelX: " + String(aX));
  Serial.println("accelY: " + String(aY));
  Serial.println("accelZ: " + String(aZ));
  Serial.println("accelSqrt: " + String(aSqrt));

  //Find force
  force_x = massOfBike * aX;
  force_y = massOfBike * aY;
  force_z = massOfBike * aZ;
  
  Serial.println("force_x: " + String(force_x));
  Serial.println("force_y: " + String(force_y));
  Serial.println("force_z: " + String(force_z));

  //integrate towards position
  secondsSoFar = millis()/1000;
  pos_x = aX * secondsSoFar * secondsSoFar * 0.5;
  pos_y = aY * secondsSoFar * secondsSoFar * 0.5;
  pos_z = aZ * secondsSoFar * secondsSoFar * 0.5;
  Serial.println("pos_x: " + String(pos_x));
  Serial.println("pos_y: " + String(pos_y));
  Serial.println("pos_z: " + String(pos_z));


  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  Serial.println("gyroX: " + String(gX));
  Serial.println("gyroY: " + String(gY));
  Serial.println("gyroZ: " + String(gZ));

  mySensor.magUpdate();
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
  mDirection = mySensor.magHorizDirection();
  Serial.println("magX: " + String(mX));
  Serial.println("maxY: " + String(mY));
  Serial.println("magZ: " + String(mZ));
  Serial.println("horizontal direction: " + String(mDirection));




  

  Serial.println("at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
  delay(500);

  
}
