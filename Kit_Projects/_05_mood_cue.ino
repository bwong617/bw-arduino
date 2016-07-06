#include <Servo.h>

Servo myServo;

int const potPin = A0;
int potVal;
int angle;

void setup() {
  Serial.begin(9600);
  myServo.attach(9);
}

void loop() {
  //retrieve ADC value (0-1023) from potentiometer
  potVal = analogRead(potPin);
  //map the ADC reading (0-1023) to the desired servo angle (0-179)
  angle = map(potVal, 0, 1023, 0, 179);
  
  Serial.print("potVal: ");
  Serial.print(potVal);
  Serial.print("\t angle: ");
  Serial.println(angle);
  
  myServo.write(angle);
  delay(15);
}
