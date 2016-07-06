#include <CapacitiveSensor.h>

CapacitiveSensor capSensor = CapacitiveSensor(4,2);

int threshold = 1000;
const int LEDpin = 12;

void setup() {
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
}

void loop() {
  long sensorVal = capSensor.capacitiveSensor(30);
  Serial.println(sensorVal);
  
  if (sensorVal > threshold){
    digitalWrite(LEDpin, HIGH);
  }
  else{
    digitalWrite(LEDpin, LOW);
  }
  delay(10);
}
