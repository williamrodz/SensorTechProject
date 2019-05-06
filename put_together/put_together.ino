const int input = A0 ;    //naming pin 2 as ‘pwm’ variable 
const int ledPin =  LED_BUILTIN;// the number of the LED pin //pin 13
int inPin = 12; //pin 12 is output of sensor
int val = 0;
const int buzzer = 10; //buzzer to arduino pin 10


void setup()
{
     pinMode(A0,INPUT_PULLUP) ;  //setting pin A0 as input 
     pinMode(ledPin, OUTPUT); //set the LED pin to output
     pinMode(inPin, INPUT);    // sets the digital pin 12 as input
     pinMode(buzzer, OUTPUT); // Set buzzer - pin 10 as an output

     Serial.begin(115200);
}
void loop()
{
     float initValue  = analogRead(0) ;    //reading analog voltage and storing it in an integer 
     float sensorValue = initValue * (5.0 / 1023.0);
     Serial.println();  
     //Serial.print(sensorValue);

     if (sensorValue< 2.2) {
      int IR_decision =0;
      Serial.print(IR_decision);
     }
     else {
      int IR_decision = 1;
      Serial.print(IR_decision);      
     }

     val = digitalRead(inPin);   // read the input pin
     if (val == HIGH){
      digitalWrite(ledPin, LOW); 
      } 
      else{
      digitalWrite(ledPin, HIGH);
      }

     tone(buzzer, 1000); // Send 1KHz sound signal...
     delay(1000);        // ...for 1 sec
     noTone(buzzer);     // Stop sound...
     delay(1000);        // ...for 1sec
  
  
    
     //Clear: around 3.41V
     //Blocked: below 1.2V (ranges depending on how far)
}
