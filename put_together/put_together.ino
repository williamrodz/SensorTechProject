const int input = A0 ;    //naming pin 2 as ‘pwm’ variable 
// ^ dont know what this does


const int dataReadPeriod = 500; //in milliseconds
// Moving Average
const int numOfForcesToSave = 100; // saves 50 seconds (100 readings/2 readings per sec) 
float savedForces[numOfForcesToSave];
float movingAverageForce;
int currentIndex = 0;
bool movingAverageCalculatedYet = false;
bool IMUAlert = false;
float forceThreshold = 15.0;


void recordNewForce (float newForce)
{
  Serial.println("New force is " + String(newForce));
  Serial.println("IMUAlert is " + String(IMUAlert));

  
  if (currentIndex < numOfForcesToSave) 
  {
    savedForces[currentIndex] = newForce;
  }
  else
  {
    savedForces[currentIndex % numOfForcesToSave] = newForce;

    if (movingAverageCalculatedYet){
      const float difference = abs(movingAverageForce - newForce);
      if (difference > forceThreshold){
        Serial.println("ALERT:passed force threshold");
        IMUAlert = true;
      }
    }
    movingAverageForce = getAverage(savedForces);
    movingAverageCalculatedYet = true;
  }
}

float getAverage(float inputArray[])
{
    float sum = 0;
    for (int i = 0; i <= sizeof(inputArray); i++)
    {
      sum = sum + inputArray[i];
    }
    return sum/sizeof(inputArray);
}

const int ledPin =  LED_BUILTIN;// the number of the LED pin //pin 13

// Hall Effect
int hallEffectOutputPin = 12; //pin 12 is output of hall effect sensor
int hallEffectVoltage = 0;


// IMU
#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN A4 //Green Cable of IMU goes to specified pin
#define SCL_PIN A5 //Silver Calble of IMU goes to specified pin
#endif

// 3.3V is VDD for IMU
MPU9250 mySensor;

uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float force_x, force_y, force_z;
float pos_x, pos_y, pos_z;
float secondsSoFar;
float massOfBike;


// Buzzer 
const int buzzerPin = 10; //buzzer to arduino pin 10
bool systemArmed = false;
bool triggerAlarm = false;


void setup()
{
     pinMode(A0,INPUT_PULLUP) ;  //setting pin A0 as input 
     pinMode(ledPin, OUTPUT); //set the LED pin to output hall effect detection
     pinMode(hallEffectOutputPin, INPUT);    // sets the digital pin 12 as input for hall effect sensor
     pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 10 as an output

    // IMU Setup  
  massOfBike = 12.7; //average mass of bike in kg
  while(!Serial);
  Serial.begin(115200); //begin serial communication at specified channel
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

  // We can set our own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  sensorId = mySensor.readId();

}
void loop()
{
     // IR LED reading
     float initValue  = analogRead(0) ;    //reading analog voltage and storing it in an integer 
     float sensorValue = initValue * (5.0 / 1023.0);
     Serial.println();  
     //Serial.print(sensorValue);

     if (sensorValue< 2.2) {
      int IR_decision = 0;
      Serial.println("IR_decision is "+ String(IR_decision));
     }
     else {
      int IR_decision = 1;
      Serial.println("IR_decision is "+ String(IR_decision));      
     }

      //Hall Effect
     hallEffectVoltage = digitalRead(hallEffectOutputPin);   // read the hallEffectOutputPin
     if (hallEffectVoltage == HIGH){
        Serial.println("Hall Effect is 0");
        digitalWrite(ledPin, LOW); 
      } 
      else{
        Serial.println("Hall Effect is 1");
        digitalWrite(ledPin, HIGH);
      }

     // IMU
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
      float xyzFmagnitude = sqrt(force_x*force_x + force_y*force_y+force_z*force_z);
      recordNewForce(xyzFmagnitude);
      Serial.println("Force Magnitue:"+ String(xyzFmagnitude));
 
      secondsSoFar = millis()/1000; 
//integrate towards position ----> decided that would be too hard to calculate due 2 need to remove gravity
    
    
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
      delay(dataReadPeriod);     

      
     //Buzzer
    if ( IMUAlert){
     tone(buzzerPin, 1000); // Send 1KHz sound signal...
     delay(1000);        // ...for 1 sec
     noTone(buzzerPin);     // Stop sound...
     delay(1000);        // ...for 1sec
    }
     
     //Clear: around 3.41V
     //Blocked: below 1.2V (ranges depending on how far)
}
