const int controlPin1 = 2;
const int controlPin2 = 4;
const int enablePin = 3;
const int dirSwitchPin = 5;
const int powerSwitchPin = 6;
const int potPin = A0;

int dirSwitchState = 0;
int dirSwitchState_prev = 0;
int powerSwitchState = 0;
int powerSwitchState_prev = 0;

int motorEnabled = 0;
int motorSpeed = 0;
int motorDir = 1;

void setup() {
  Serial.begin(9600);
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(dirSwitchPin, INPUT);
  pinMode(powerSwitchPin, INPUT);
  digitalWrite(enablePin, LOW);
}

void loop() {
  powerSwitchState = digitalRead(powerSwitchPin);
  delay(1);
  dirSwitchState = digitalRead(dirSwitchPin);
  motorSpeed = analogRead(potPin)/4;
  
  if(powerSwitchState != powerSwitchState_prev){
    if(powerSwitchState == HIGH){
      motorEnabled = !motorEnabled;
      Serial.println("--POWER--");
    }
  }
  if(dirSwitchState != dirSwitchState_prev){
    if(dirSwitchState == HIGH){
      motorDir = !motorDir;
      Serial.println("---DIR---");
    }
  }
  
  if(motorDir == 1){
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
  }
  else{
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
  }
  
  if(motorEnabled == 1)
    analogWrite(enablePin, motorSpeed);
  else
    analogWrite(enablePin, 0);
  
  dirSwitchState_prev = dirSwitchState;
  powerSwitchState_prev = powerSwitchState;
  
  Serial.print("motorEnabled: ");
  Serial.print(motorEnabled);
  Serial.print("\t motorSpeed: ");
  Serial.print(motorSpeed);
  Serial.print("\t motorDir: ");
  Serial.print(motorDir);
  Serial.print("\t dirSwitchState: ");
  Serial.print(dirSwitchState);
  Serial.print("\t dirSwitchState_prev: ");
  Serial.print(dirSwitchState_prev);
  Serial.print("\t powerSwitchState: ");
  Serial.print(powerSwitchState);
  Serial.print("\t powerSwitchState_prev: ");
  Serial.println(powerSwitchState_prev);
}
