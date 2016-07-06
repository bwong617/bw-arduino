//sensor input pins
const int SSpinR = A0;
const int SSpinG = A1;
const int SSpinB = A2;

//LED output pins
const int LEDpinR = 11;
const int LEDpinG = 10;
const int LEDpinB = 9;

//sensor color values
int SSvalR = 0;
int SSvalG = 0;
int SSvalB = 0;

//LED color values
int LEDvalR = 0;
int LEDvalG = 0;
int LEDvalB = 0;

void setup() {
  //open serial port; baud rate = 9600
  Serial.begin(9600);
  
  //initialize output pins
  pinMode(LEDpinR, OUTPUT);
  pinMode(LEDpinG, OUTPUT);
  pinMode(LEDpinB, OUTPUT); 
}

void loop() {

  //retrieve sensor color data
  SSvalR = analogRead(SSpinR);
  delay(5);    //ADC takes one millisecond to do work
  SSvalG = analogRead(SSpinG);
  delay(5);
  SSvalB = analogRead(SSpinB);
  
  Serial.print("[SS Values] \t R: ");
  Serial.print(SSvalR);
  Serial.print("\t G: ");
  Serial.print(SSvalG);
  Serial.print("\t B: ");
  Serial.println(SSvalB);

  //convert sensor data (0-1023) to LED duty cycle (0-255)
  LEDvalR = SSvalR / 4;
  LEDvalG = SSvalG / 4;
  LEDvalB = SSvalB / 4;
  
  Serial.print("[LED Values] \t R: ");
  Serial.print(LEDvalR);
  Serial.print("\t G: ");
  Serial.print(LEDvalG);
  Serial.print("\t B: ");
  Serial.println(LEDvalB);
  
  analogWrite(LEDpinR, LEDvalR);
  analogWrite(LEDpinG, LEDvalG);
  analogWrite(LEDpinB, LEDvalB);
  
}
