int switchState1 = 0;    //toggles the mode
int switchState2 = 0;    //increases the blink speed
int blinkDelay = 400;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);     //switch1
  pinMode(13, INPUT);    //switch2
  pinMode(3, OUTPUT);    //green LED
  pinMode(7, OUTPUT);    //red LED1
  pinMode(11, OUTPUT);   //red LED2
}

void loop() {
  // put your main code here, to run repeatedly:
  switchState1 = digitalRead(2);
  switchState2 = digitalRead(13);
  
  if (switchState1 == LOW){
    digitalWrite(3, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(11, LOW);
  }
  else{
    
    if (switchState2 == LOW)
      blinkDelay = 400;
    else
      blinkDelay = 200;
    
    digitalWrite(3, LOW);
    digitalWrite(7, LOW);
    digitalWrite(11, HIGH);
    
    delay(blinkDelay);
    
    digitalWrite(7, HIGH);
    digitalWrite(11, LOW);
    
    delay(blinkDelay);
  }
  
}
