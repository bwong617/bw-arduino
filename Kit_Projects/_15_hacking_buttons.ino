const int optoPin = 2;

void setup() {
  pinMode(optoPin, OUTPUT);
}

void loop() {
  digitalWrite(optoPin, HIGH);
  delay(100);
  digitalWrite(optoPin, LOW);
  delay(3000);
}
