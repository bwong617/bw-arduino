const int photoPin = A0;
const int piezoPin = 12;
const int redPin = 9;
const int greenPin = 6;

int photoVal;
int photoLow = 0;
int photoHigh = 1023;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  //calibration (adjust photoHigh and photoLow)
  digitalWrite(redPin, HIGH);   //calibration indicator
  digitalWrite(greenPin, LOW);
  
  while (millis() < 5000){      //run loop till 5 seconds after power-on/reset
    photoVal = analogRead(photoPin);
    if (photoVal > photoHigh)
      photoHigh = photoVal;
    if (photoVal < photoLow)
      photoLow = photoVal;
  }
  
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
}

void loop() {
  //map light reading to sound pitch (50-4000 Hz)
  photoVal = analogRead(photoPin);
  int pitch = map(photoVal, photoLow, photoHigh, 50, 2000);
  
  //play tone (output pin, pitch, length of note (ms))
  tone(piezoPin, pitch, 20);
  
  delay(10);
}
