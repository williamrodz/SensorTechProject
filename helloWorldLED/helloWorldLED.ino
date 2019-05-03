


  
// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin
int inPin = 12;
int val = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(ledPin, OUTPUT); //set the LED pin to output
  pinMode(inPin, INPUT);    // sets the digital pin 12 as input


}

void loop() {
  // put your main code here, to run repeatedly:
  // here is where you'd put code that needs to be running all the time.

//  val = digitalRead(inPin);   // read the input pin
//  if (val == HIGH){
//      digitalWrite(ledPin, LOW);
//  } else{
//      digitalWrite(ledPin, HIGH);
//  }
//  
//  digitalWrite(ledPin,HIGH);
//  delay(1000);
//  digitalWrite(ledPin,LOW);
//  delay(5000);

}
