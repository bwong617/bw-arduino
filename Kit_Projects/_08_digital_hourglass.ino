const int tiltPin = 8;

int tiltState = 0;
int prevTiltState = 0;
int nextLED = 2;

unsigned long prevTime = 0;
long interval = 50;

void setup() {
  for(int i = 2; i <= 7; i++)
    pinMode(i, OUTPUT);
  pinMode(tiltPin, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  if(currentTime - prevTime > interval){
    prevTime = currentTime;
    digitalWrite(nextLED, HIGH);
    nextLED++;
    if(nextLED == 9)
      resetLED();
  }
  tiltState = digitalRead(tiltPin);
  if(tiltState != prevTiltState){
    resetLED();
    prevTime = currentTime;
  }
  prevTiltState = tiltState;
}

void resetLED() {
  for(int i = 2; i <= 7; i++)
    digitalWrite(i, LOW);
  nextLED = 2;
}
