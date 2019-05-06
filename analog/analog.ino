const int input = A0 ;    //naming pin 2 as ‘pwm’ variable 

void setup()
{
     pinMode(A0,INPUT_PULLUP) ;  //setting pin A0 as input
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
    
     //Clear: around 3.41V
     //Blocked: below 1.2V (ranges depending on how far)

     //1V = 200
     //2V = 400
     //3V = 600
     //4V = 800
     //5V = 1000

     //4.9 = 990
     //4.8 = 970
     //4.7 = 950
     //4.6 = 930
     //4.5 = 910
     //4.4 = 890
     //4.3 = 870
     //4.2 = 850
     //4.1 = 830

     //Rounds to nearest voltage
  //analogWrite(pwm,adc) ; 
}
