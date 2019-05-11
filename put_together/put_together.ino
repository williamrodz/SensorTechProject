
// Pins
// IR-LED input -> Analog 0
// Hall Effect Output -> Digital 12
// SDA Cable of IMU -> Analog 5
// SCL Cable of IMU -> Analog 4
// Buzzer red input -> Digital 4
// Switch Output -> Digital Pin 7
// Switch Input -> Digital Pin 10

// Power
// Everything except IMU uses 5V. IMU uses 3.3V

// Debugging
bool debugAcceleration = false;
bool debugForce = true;
bool debugGyro = false;
bool debugMagnetic = false;


// Switch
bool ARMED = false;
int switchOutputPin = 7;
int switchVinPin = 10;

// Alarm Decision
bool IRSensorTriggered = false;
bool magnetSensorTriggered = false;
bool IMUTriggered = false;


// IR-LED
//const int input = A0 ;    //naming pin 2 as ‘pwm’ variable
int IROutputPin = A0;

// Hall effect
int hallEffectOutputPin = 12;
int hallEffectVoltage = 0;

// IMU
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
float massOfBike = 12.7; //average mass of bike in kg


// Buzzer 
const int buzzerPin = 4;
bool systemArmed = false;
bool triggerAlarm = false;


const int dataReadPeriod = 500; //in milliseconds

// Moving Average
const int numOfForcesToSave = 100; // saves 50 seconds (100 readings/2 readings per sec) 
float savedForces[numOfForcesToSave];


float movingAverageForce;
int currentIndex = 0;
bool movingAverageCalculatedYet = false;
float forceDifferenceThreshold = 7;


void recordNewForce (float newForce)
{
  //Serial.println("New force is " + String(newForce));
  Serial.println("IMUTriggered is " + String(IMUTriggered));

  
  if (currentIndex < numOfForcesToSave) 
  {
    Serial.println("at index " +String(currentIndex));
    savedForces[currentIndex] = newForce;
  }
  else
  {
    Serial.println("Refilling at " +String(currentIndex));

    savedForces[currentIndex % numOfForcesToSave] = newForce;

    if (movingAverageCalculatedYet){
      const float difference = abs(movingAverageForce - newForce);
      Serial.println("movingAverageForce is "+ String(movingAverageForce));
      Serial.println("Difference is "+ String(difference));
      if (difference > forceDifferenceThreshold){
        Serial.println("ALERT:passed force threshold");
        IMUTriggered = true;
      }
    }
    movingAverageForce = getAverage(savedForces);
    movingAverageCalculatedYet = true;
  }
  currentIndex++;
}

float getAverage(float inputArray[])
{
    float sum = 0;
    for (int i = 0; i < sizeof(inputArray); i++)
    {
      sum = sum + inputArray[i];
    }
    return sum/sizeof(inputArray);
}


void toggleARMED()
{
  ARMED = !ARMED;
}

void setup()
{
     pinMode(IROutputPin,INPUT_PULLUP) ;  //setting pin A0 as input for IR-LED sensor
     pinMode(hallEffectOutputPin, INPUT);    // sets the digital pin 12 as input for hall effect sensor
     pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 10 as an output
     pinMode(switchVinPin,OUTPUT);
     pinMode(switchOutputPin,INPUT);


    // IMU Setup  
  while(!Serial);
  Serial.begin(115200); //begin serial communication at specified channel
  //Serial.println("Started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
#else
  Wire.begin();
#endif

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  sensorId = mySensor.readId();

}
void loop()
{

    // Switch
     int switchVoltage = digitalRead(switchOutputPin);   // read the hallEffectOutputPin
     Serial.println("switchVoltage" + String(switchVoltage));
     if (switchVoltage == HIGH){
        Serial.println("Switch: 1");
      } 
      else{
        Serial.println("Switch: 0");
        toggleARMED();
      }

  
     // IR LED reading
     float initValue  = analogRead(0) ;    //reading analog voltage and storing it in an integer 
     float sensorValue = initValue * (5.0 / 1023.0);
     //Serial.println();  

     if (sensorValue < 2.2) {
      int IR_decision = 0;
      Serial.println("IR: "+ String(IR_decision));
     }
     else {
      int IR_decision = 1;
      Serial.println("IR:  "+ String(IR_decision));      
     }

      //Hall Effect
     hallEffectVoltage = digitalRead(hallEffectOutputPin);   // read the hallEffectOutputPin
     if (hallEffectVoltage == HIGH){
        Serial.println("Hall: 0");
      } 
      else{
        Serial.println("Hall: 1");
      }

     // IMU
    Serial.println("sensorId: " + String(sensorId));
    
      mySensor.accelUpdate();
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      aSqrt = mySensor.accelSqrt();
      if (debugAcceleration){
        Serial.println("accelX: " + String(aX));
        Serial.println("accelY: " + String(aY));
        Serial.println("accelZ: " + String(aZ));
        Serial.println("accelSqrt: " + String(aSqrt));
      }

      //Find force
      force_x = massOfBike * aX;
      force_y = massOfBike * aY;
      force_z = massOfBike * aZ;
      float xyzFmagnitude = sqrt(force_x*force_x + force_y*force_y+force_z*force_z);
      recordNewForce(xyzFmagnitude);
      if (debugForce){
        Serial.println("Force Magnitue:"+ String(xyzFmagnitude));

      }
 
      secondsSoFar = millis()/1000;     
    
      mySensor.gyroUpdate();
      gX = mySensor.gyroX();
      gY = mySensor.gyroY();
      gZ = mySensor.gyroZ();
      if (debugGyro){
        Serial.println("gyroX: " + String(gX));
        Serial.println("gyroY: " + String(gY));
        Serial.println("gyroZ: " + String(gZ));
      }

    
      mySensor.magUpdate();
      mX = mySensor.magX();
      mY = mySensor.magY();
      mZ = mySensor.magZ();
      mDirection = mySensor.magHorizDirection();
      
      if (debugMagnetic){
        Serial.println("magX: " + String(mX));
        Serial.println("maxY: " + String(mY));
        Serial.println("magZ: " + String(mZ));
        Serial.println("horizontal direction: " + String(mDirection)); 
      }

     
      Serial.println("at " + String(millis()) + "ms");
      Serial.println(""); // Add an empty line
          

      bool allowAlarm = false;
     //Buzzer
    if (IMUTriggered && allowAlarm){
     tone(buzzerPin, 1000); // Send 1KHz sound signal...
     delay(1000);        // ...for 1 sec
     noTone(buzzerPin);     // Stop sound...
     delay(1000);        // ...for 1sec
    }
    delay(dataReadPeriod); 
     
     //Clear: around 3.41V
     //Blocked: below 1.2V (ranges depending on how far)
     
  // We can set our own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}
