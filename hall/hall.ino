


  
// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin //pin 13
int inPin = 12; //pin 12 is output of sensor
int val = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(ledPin, OUTPUT); //set the LED pin to output
  pinMode(inPin, INPUT);    // sets the digital pin 12 as input

  while(!Serial);
  Serial.begin(115200); //begin serial communication at specified channel
  Serial.println("started");


}

void loop() {
  // put your main code here, to run repeatedly:
  // here is where you'd put code that needs to be running all the time.

  val = digitalRead(inPin);   // read the input pin
  if (val == HIGH){
      //digitalWrite(ledPin, LOW);
      //Serial.println("HIGH");
  } else{
      //digitalWrite(ledPin, HIGH);
      Serial.println("Detected Magnet South");
  }
  
  //digitalWrite(ledPin,HIGH);
  //delay(1000);
  //digitalWrite(ledPin,LOW);
  //delay(5000);

}
