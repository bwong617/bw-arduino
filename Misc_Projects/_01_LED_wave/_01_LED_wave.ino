static const int piezoPin = 12;
static const int photoPin = A0;

static int photoVal;
static int photoLow = 0;
static int photoHigh = 1023;

//LED pin sequence
int ledPin[] = {5,6,7,8,9,10,11};
//length of LED pin array
const int ledPinSize = 7;
//length of LED wave
const int wavelength = 3;
//speed of LED wave
int interval;
//const int interval = 50;

//setup
void setup() {
  //initialize serial monitor (baud rate 9600)
  Serial.begin(9600);
  //initialize LED pins as outputs
  for(int i = 0; i < ledPinSize; i++){
    pinMode(ledPin[i], OUTPUT);
  }
  
  //calibration indicator on
  digitalWrite(ledPin[0], HIGH);
  //run loop till 5 seconds after power-on/reset
  while (millis() < 3000){      
    photoVal = analogRead(photoPin);
    if (photoVal > photoHigh)
      photoHigh = photoVal;
    if (photoVal < photoLow)
      photoLow = photoVal;
  }
  digitalWrite(ledPin[0], LOW);
}

//wave sequence
void loop() {
  forward();
  forward();
  reverse();
  reverse();
}

//forward wave
void forward() {
  Serial.println("FORWARD");
  for(int i = 0; i < ledPinSize+wavelength; i++){
    if(i < ledPinSize)
      digitalWrite(ledPin[i], HIGH);
    if(i >= wavelength)
      digitalWrite(ledPin[i-wavelength], LOW);
    Serial.println(i);
    interval = photo_piezo();
    delay(interval);
  }
}

//reverse wave
void reverse() {
  Serial.println("REVERSE");
  for(int i = (ledPinSize-1); i > 0-wavelength-1; i--){
    if(i >= 0)
      digitalWrite(ledPin[i], HIGH);
    if(i <= (ledPinSize-wavelength-1))
      digitalWrite(ledPin[i+wavelength], LOW);
    Serial.println(i);
    interval = photo_piezo();
    delay(interval);
  }
}

int photo_piezo() {
  //retrieve light reading from photoresistor
  int photoVal = analogRead(photoPin);
  //map light reading to wave speed
  int waveSpeed = map(photoVal, photoLow, photoHigh, 10, 100);
  //map light reading to sound pitch (50-4000 Hz)
  int pitch = map(photoVal, photoLow, photoHigh, 100, 2000);
  //play tone (output pin, pitch, length of note (ms))
  tone(piezoPin, pitch, 20);
  
  return(waveSpeed);
}
