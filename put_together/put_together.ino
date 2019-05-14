
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
bool debugSystemArmed = false;
bool debugIMUTriggered = false;
bool debugIRTriggered = false;
bool debugIR_RPP = false;
bool debugIRVoltage = false;
bool debugHallTriggered = false;
bool debugHall_RPP = true;
bool debugAcceleration = false;
bool debugForce = false;
bool debugGyro = false;
bool debugMagnetic = false;
bool viewTime = false;

// Switch
bool ARMED = false;
int switchOutputPin = 7;
int switchVinPin = 10;


// Alarm Decision
bool IRSensorTriggered = false;
bool hallSensorTriggered = false;
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


const int dataReadPeriod = 100; //in milliseconds

// Moving Average
const int numOfDataPointsToSave = 100; // saves 50 seconds (100 readings/2 readings per sec) 
float savedForces[numOfDataPointsToSave];
// saved wheel detections
int savedHallDetections[numOfDataPointsToSave];
int savedIRDetections[numOfDataPointsToSave];


float movingAverageForce;
bool movingAverageCalculatedYet = false;
float forceDifferenceThreshold = 6;


int i_forces = 0;
void recordNewForce (float newForce)
{
 
  if (i_forces < numOfDataPointsToSave) 
  {
    //Serial.println(0);
    savedForces[i_forces] = newForce;
  }
  else
  {
    //Serial.println("Refilling at " +String(i_forces));

    savedForces[i_forces % numOfDataPointsToSave] = newForce;

    if (movingAverageCalculatedYet){
      const float difference = abs(movingAverageForce - newForce);
      //Serial.println(String(difference));
      
      if (difference > forceDifferenceThreshold){
        Serial.println("ALERT:passed force threshold");
        IMUTriggered = true;
      }
    }
    movingAverageForce = getAverage(savedForces);
    movingAverageCalculatedYet = true;
  }
  i_forces++;
}


const int HALL_RPP_LIMIT = 2;

int i_hall = 0;
void recordNewHallDetection(int newHallValue)
{
  if (i_hall < numOfDataPointsToSave)
  {
    savedHallDetections[i_hall] = newHallValue;
  }
  else
  {
    savedHallDetections[i_hall % numOfDataPointsToSave] = newHallValue;
    int HALL_RPP = getSumOfArray(savedHallDetections);
    if (debugHall_RPP)
    {
      //Serial.println("HALL_RPP: "+String(HALL_RPP));
      Serial.println(String(HALL_RPP));      
    }
    if (HALL_RPP >= HALL_RPP_LIMIT)
    {
      hallSensorTriggered = true;
    }
  }
  i_hall++;
}

int i_IR = 0;
const int IR_RPP_LIMIT = 3;
void recordNewIRDetection(int newIRValue)
{
  if (i_IR < numOfDataPointsToSave){
    savedIRDetections[i_IR] = newIRValue;
  }
  else
  {
    savedIRDetections[i_IR % numOfDataPointsToSave] = newIRValue;
    int IR_RPP = getSumOfArray(savedIRDetections);
    if (debugIR_RPP)
    {
      //Serial.println("IR_RPP: "+String(IR_RPP));
      Serial.println(String(IR_RPP));    
    }
    
    if (IR_RPP >= IR_RPP_LIMIT)
    {
      IRSensorTriggered = true;
    }    
  }
  i_IR++;
}


int getSumOfArray(int integerArray[])
{
  int sum = 0;
  int n = numOfDataPointsToSave;//sizeof(integerArray) / sizeof(integerArray[0]);
  for (int i = 0; i < n ; i++)
  { 
      sum += integerArray[i];
  }
  return sum;
}

void setArrayToZero(int integerArray[])
{
  int sum = 0;
  int n = numOfDataPointsToSave;//sizeof(integerArray) / sizeof(integerArray[0]);
  for (int i = 0; i < n ; i++)
  { 
      integerArray[i] = 0;
  }
}


// return the average of an inputted array
float getAverage(float inputArray[])
{
    float sum = 0;
    for (int i = 0; i < sizeof(inputArray); i++)
    {
      sum = sum + inputArray[i];
    }
    return sum/sizeof(inputArray);
}

// Changes alarm from armed to disarmed and vice-versa
void toggleARMED()
{
  ARMED = !ARMED;
  updateALARMEDled();
}

void updateALARMEDled()
{
  if (ARMED)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void checkIfShouldDisarm()
{
  int switchVoltage = digitalRead(switchOutputPin);
  if (switchVoltage == LOW)
  {
    ARMED = false;
    IMUTriggered = false;
    IRSensorTriggered = false;
    hallSensorTriggered = false;
    updateALARMEDled();
    setArrayToZero(savedHallDetections);
    setArrayToZero(savedIRDetections);
  }
}

void loudAlarm()
{
  // buzz
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  noTone(buzzerPin);     // Stop sound...
  delay(100);
  
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(100);
  noTone(buzzerPin);     // Stop sound...
  delay(100);
  
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(1000);
  noTone(buzzerPin);     // Stop sound...     
  
  delay(2000);        // ...for 2sec
}

void lowAlarm()
{
  // buzz
  tone(buzzerPin, 750); // Send 1KHz sound signal...
  delay(200);        // ...for 1 sec
  noTone(buzzerPin);     // Stop sound...
  delay(100);  
}

// ----------SETUP BLOCK BEGINS HERE ----------
// -----(code that runs right before loop) ----------


void setup()
{
     pinMode(IROutputPin,INPUT_PULLUP) ;  //setting pin A0 as input for IR-LED sensor
     pinMode(hallEffectOutputPin, INPUT);    // sets the digital pin 12 as input for hall effect sensor
     pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 10 as an output
     pinMode(switchVinPin,OUTPUT);
     digitalWrite(switchVinPin,HIGH);
     pinMode(switchOutputPin,INPUT);
     pinMode(LED_BUILTIN, OUTPUT);


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

// ----------LOOP BLOCK BEGINS HERE ----------
// -----(code that runs every dataReadPeriod) ----------


void loop()
{
    // Switch
    if (debugSystemArmed)
    {
     Serial.println("Is system armed?: "+String(ARMED));
    }
    if (debugIMUTriggered)
    {
     Serial.println("IMUTriggered is " + String(IMUTriggered));
      
    }
    if (debugHallTriggered)
    {
     Serial.println("hallSensorTriggered is " + String(hallSensorTriggered));      
    }
    if (debugIRTriggered)
    {
     Serial.println("IRSensorTriggered is " + String(IRSensorTriggered));      
    }


     int switchVoltage = digitalRead(switchOutputPin);   // read the hallEffectOutputPin
     if (switchVoltage == HIGH){
        //Serial.println("Switch: 1");
      } 
      else{
        //Serial.println("Switch: 0");
        toggleARMED();
      }

  
     // IR LED reading
     float initValue  = analogRead(0) ;    //reading analog voltage and storing it in an integer 
     float sensorValue = initValue * (5.0 / 1023.0);
     if (debugIRVoltage)
     {
      Serial.println(sensorValue);      
     }
     
     if (sensorValue < 2.2) {
      int IR_decision = 0;
      //Serial.println("IR: "+ String(IR_decision));
      recordNewIRDetection(1); //it's opposite, because IR=1 means spoke has not crossed IR LED
     }
     else {
      int IR_decision = 1;
      //Serial.println("IR:  "+ String(IR_decision));
      recordNewIRDetection(0);   
     }

      //Hall Effect
     hallEffectVoltage = digitalRead(hallEffectOutputPin);   // read the hallEffectOutputPin
     if (hallEffectVoltage == HIGH){
        //Serial.println("Hall: 0");
        recordNewHallDetection(0);
      } 
      else{
        //Serial.println("Hall: 1");
        recordNewHallDetection(1);
      }

     // IMU
    //Serial.println("sensorId: " + String(sensorId));
    
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
      if (!movingAverageCalculatedYet)
      {
        recordNewForce(xyzFmagnitude);
      }
      else
      {
        //Serial.println("movingAverageForce is "+ String(movingAverageForce));
        const float difference = abs(movingAverageForce - xyzFmagnitude);
        if (debugForce)
        {
          Serial.println("Difference is "+String(difference));
          Serial.println(String(difference));
          
        }
        if (difference > forceDifferenceThreshold)
        {
          IMUTriggered = true;
        }
      }
      if (debugForce)
      {
        Serial.println("Force Magnitude:"+ String(xyzFmagnitude));
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

      if (viewTime)
      {
        Serial.println("at " + String(millis()) + "ms"); 
      }
      Serial.println(""); // Add an empty line
          

      bool allowAlarm = true;
     //Buzzer
    bool twoAlarmsTriggered = (IMUTriggered && (hallSensorTriggered || IRSensorTriggered)) || (hallSensorTriggered && (IMUTriggered || IRSensorTriggered)) || (IRSensorTriggered && (IMUTriggered || hallSensorTriggered));
    bool oneAlarmTriggered = IMUTriggered || hallSensorTriggered || IRSensorTriggered;
    if (ARMED && twoAlarmsTriggered && allowAlarm)
    {
     loudAlarm();
     checkIfShouldDisarm();
    }
    else if (ARMED && oneAlarmTriggered && allowAlarm)
    {
     lowAlarm();
     checkIfShouldDisarm();

    } 
    delay(dataReadPeriod); 
     
     //Clear: around 3.41V
     //Blocked: below 1.2V (ranges depending on how far)
     
  // We can set our own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}
