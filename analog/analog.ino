const int input = A0 ;    //naming pin 2 as ‘pwm’ variable 
const int adc = 0 ;   //naming pin 0 of analog input side as ‘adc’

void setup()
{
     pinMode(A0,INPUT_PULLUP) ;  //setting pin A0 as input
     Serial.begin(115200);
}
void loop()
{
     int adc  = analogRead(0) ;    //reading analog voltage and storing it in an integer 
     //adc = map(adc, 0, 1023, 0, 255);
     Serial.println();  
     Serial.print(adc);

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
/*
----------map funtion------------the above funtion scales the output of adc, which is 10 bit and gives values btw 0 to 1023, in values btw 0 to 255 form analogWrite funtion which only receives  values btw this range
*/
     //analogWrite(pwm,adc) ; 
}
