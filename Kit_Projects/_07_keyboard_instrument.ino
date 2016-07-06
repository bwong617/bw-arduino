const int keyPin = A0;
const int piezoPin = 12;

//reference:
//int buttons[6];                    //initialize array with 6 integers
//int buttons[0] = 2;                //assign value to first element

int notes[] = {262,294,330,349};     //C,D,E,F

void setup() {
  Serial.begin(9600);
}

void loop() {
  int keyVal = analogRead(keyPin);
  
  Serial.println(keyVal);
  
  if(keyVal == 1023)
    tone(piezoPin, notes[0]);
  else if(keyVal >= 990 && keyVal <= 1010)
    tone(piezoPin, notes[1]);
  else if(keyVal >= 505 && keyVal <= 515)
    tone(piezoPin, notes[2]);
  else if(keyVal >= 5 && keyVal <= 10)
    tone(piezoPin, notes[3]);
  else
    noTone(piezoPin);
}
