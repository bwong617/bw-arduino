volatile int state=LOW; //this is the variable that will be changed if an interrupt is detected
int inputPin=4; //Polling will check this pin
int i=0; //For loop counting variable
void setup()
{
  attachInterrupt(0,brachiya,CHANGE); //Interrupt 0 is mapped to pin 2, signal an interrupt on a change to pin 2
  Serial.begin(9600);
}

void loop()
{
      for(i=1;i<=15;i++)
      {
        if(state == HIGH)
        {
          Serial.println (i);
        }
        else
        {
          Serial.println ("Interrupt Detected stop counting");
          while(state == LOW)
          {};
          Serial.println (i);
        }
        delay(500); 
      }
    Serial.println ("Check polling");
    if(digitalRead(inputPin) == LOW)
    {
      Serial.println("Polling LOW detected: stop program");
      while(digitalRead(inputPin) == LOW)
      {};
      Serial.println("Polling Checked pin 4 found to be HIGH Start/Continue Program");
    }
      
}

  
  
    /*Poll state of pin 4, if it is high print: "Polling Checked pin 4 found to be HIGH Start/Continue Program"
    Delay for 0.5sec
    Setup for loop to count from 1 to 15
      If your interrupt variable (state) is high print 1 to 15 with a 0.5sec delay between prints statements
      If your interrupt variable (state) is LOW print: "Interrupt Detected stop counting" stop counting and wait for a change in the interrupt variable
    After you have counted to 15 print: "Check polling" and poll pin 4, if it is high repeat counting/interrupting.
    If the polled pin is LOW print: "Polling LOW detected stop program" then wait until polling pin changes*/

void brachiya() 
{
  state=!state; //change the interrupt variable from LOW to HIGH or HIGH to LOW
}
