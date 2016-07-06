#include <Servo.h>

Servo myServo;

const int piezo = A0;
const int switchPin = 2;
const int yellowLED = 8;
const int greenLED = 9;
const int redLED = 10;

int switchVal;
int knockVal;
int numKnocks = 0;
const int quietKnock = 10;
const int loudKnock = 100;
boolean locked = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(6);
  
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(switchPin, OUTPUT);
  
  digitalWrite(greenLED, HIGH);
  myServo.write(0);
  Serial.println("The box is unlocked!");
}

void loop() {
  if (locked == false){ //press button to lock
    switchVal = digitalRead(switchPin);
    if (switchVal == HIGH){
      locked = true;
      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, HIGH);
      myServo.write(90);
      Serial.println("The box is locked!");
      delay(1000);
    }
  }
  if (locked == true){ //knock thrice to unlock
    knockVal = analogRead(piezo);
    if (knockVal > 0 && numKnocks < 3){
      if (checkForKnock(knockVal) == true){
        numKnocks++;
      }
      Serial.print(3-numKnocks);
      Serial.println(" more knocks to go");
    }
    if (numKnocks >= 3){
      locked = false;
      numKnocks = 0;
      myServo.write(0);
      delay(20);
      digitalWrite(greenLED, HIGH);
      digitalWrite(redLED, LOW);
      Serial.println("The box is unlocked!");
    }
  }
}

boolean checkForKnock(int value){
  if (value > quietKnock && value < loudKnock){
    digitalWrite(yellowLED, HIGH);
    delay(50);
    digitalWrite(yellowLED, LOW);
    Serial.print("Valid knock of value ");
    Serial.println(value);
    return true;
  }
  else{
    Serial.print("Bad knock value ");
    Serial.println(value);
    return false;
  }
}
