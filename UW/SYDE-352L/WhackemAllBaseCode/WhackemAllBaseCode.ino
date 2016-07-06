////////////////////////////////////////////////////////////////////
//  Whack'em All Base Code
//  
//  Created By: Chris McClellan
//  Created On: December 21, 2015
//  Description: Created base template
//
//  Modified By: Brandon Wong
//  Modified On: March 7, 2016
//  Description: Added user-defined variables, setup code, and body
////////////////////////////////////////////////////////////////////

#include <MsTimer2.h>
#include <math.h>

// Setup Motor Controller 1 Pins
static byte PWM1_Out = 4;      // Motor Controller PWM1 assigned to Digital Pin 4 
static byte PWM2_Out = 5;      // Motor Controller PWM2 assigned to Digital Pin 5 
static byte Disable_Out = 6;   // Motor Controller Disable assigned to Digital Pin 6 
static byte En_Out = 7;        // Motor Controller Enable assigned to Digital Pin 7
static byte FB_AIn = 0;        // Motor Controller Current Feedback assigned to Analog Pin 0

// Setup Encoder Counter 1 Pins
static byte SEL1 = 38;         // Select 1 output signal assigned to Pin 38
static byte OE = 39;           // Output Enable signal assinged to Pin 39
static byte RST = 40;          // Reset output signal assigned to Pin 40
static byte SEL2 = 41;         // Select 2 output signal assinged to Pin 41

// Setup Emitter Pins for Optical Sensors 1-4 
static byte E1 = 46;           // Emmitter on Optical Sensor 1 assigned to Digital Pin 46
static byte E2 = 47;           // Emmitter on Optical Sensor 2 assigned to Digital Pin 47
static byte E3 = 48;           // Emmitter on Optical Sensor 3 assigned to Digital Pin 48
static byte E4 = 49;           // Emmitter on Optical Sensor 4 assigned to Digital Pin 49

// Setup ISR Global Variables
volatile byte go = 0;          // Interrupt flag for START
volatile byte timer_go = 0;    // Interrupt flag for timer
volatile byte Stop = 0;        // Interrupt flag for E-STOP  
volatile byte Received = 0;    // Interrupt flag for Receiver of all Optical Sensors
volatile long signed int Received_Position = 0; // Encoder position when ISR for Receiver is triggered
volatile long signed int Time_Count = 0;        // Time Count variable
volatile long signed int Received_Count = 0;    // Received Time count for when Recieved funtion is triggered 
volatile byte DONE = 0;        // Received Count flag  

// Declare Contstants
static float pi = 3.141593;

// Control Loop Period in milli seconds (1-1000)
static unsigned int Period = 2;
 
// Voltage Input (Volts)
static float V_in = 12.0; 

// Input Filter Function I(s) (assumed to be a gain)
static float I_Gain = 651.9;


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// User Defined Global Variables

/////////////////////////////////////////////////////////////////////////////////////////

// Declare Target Sequence (1, 2 , 3 or 4)
volatile byte Target_Sequence[] = {1,2,3,4,3,2,1,2,1,2};

// Declare number of targets in Target Sequence
volatile byte TS_Length = 10;

// PID Gains 
static float Kp = 0.1232;       // Set Proportioanl Gain     
static float Ki = 0;       // Set Integral Gain
static float Kd = 0.006;       // Set Derivative Gain

volatile float VtoPWM = 0;                    // Conversion factor for Volt to PWM  
volatile float KdxFreq = 0;                   // Combined gain of Kd*Freq
volatile float PeriodinSeconds = 0;           // Control Loop Period in seconds
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Read Encoder Position Function
signed long int Read_Encoder()
{
  signed long int Position;     // encoder position total
  signed long int temp0;        // temp variable used for reading in encoder value
  signed long int temp1;        // temp variable used for reading in encoder value 
  signed long int temp2;        // temp variable used for reading in encoder value
  signed long int temp3;        // temp variable used for reading in encoder value
    
  // initiate sequence to retrieve encoder counter values    
  digitalWrite(OE, HIGH);
  digitalWrite(OE, LOW);

  // retreive byte 0 from encoder counter 
  digitalWrite(SEL1, HIGH);              
  digitalWrite(SEL2, LOW);
  temp0 = PINA;               // Read in values from Port A

  // retreive byte 1 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, LOW);
  temp1 = PINA;               // Read in values from Port A
    
  // retreive byte 2 from encoder counter  
  digitalWrite(SEL1, HIGH);
  digitalWrite(SEL2, HIGH);
  temp2 = PINA;               // Read in values from Port A    
    
  // retreive byte 3 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, HIGH);
  temp3 = PINA;               // Read in values from Port A  
  
  Position = temp0 + (temp1 << 8)+ (temp2 << 16)+ (temp3 << 24);     // calculate encoder count 
  
  // end encoder counter read sequence
  digitalWrite(OE, HIGH);  
 
  return Position;
} 

//  Stop Motor Function
void Stop_Motor()
{
  // Disable Motor Controller thereby disabling Motor 
  digitalWrite(En_Out, LOW);           // Set Motor Controller Enable low to disable motor
  digitalWrite(PWM1_Out, LOW);         // Set Motor Controller PWM 1 low to disable motor
  digitalWrite(PWM2_Out, LOW);         // Set Motor Controller PWM 2 low to disable motor
  digitalWrite(Disable_Out, LOW);      // Set Motor Controller Disable low to disable motor 
}

// Main Setup Routine
void setup() 
{
  Serial.begin(115200);       // Setup serial output to 115,200 bps (bits per second)
     
  // Setup Data input pins from Encoder as inputs
  DDRA = 0;                   // Set pins 22-29 as inputs

  // Setup Motor Controller 1 pins as Outputs
  pinMode(PWM1_Out, OUTPUT);     // Setup digital pin 4 as an output
  pinMode(PWM2_Out, OUTPUT);     // Setup digital pin 5 as an output
  pinMode(Disable_Out, OUTPUT);  // Setup digital pin 6 as an output
  pinMode(En_Out, OUTPUT);       // Setup digital pin 7 as an output
  
  // Setup Encoder 1 Counter pins as Outputs
  pinMode(SEL1, OUTPUT);         // Setup digital pin 38 as an output 
  pinMode(OE, OUTPUT);           // Setup digital pin 39 as an output  
  pinMode(RST, OUTPUT);          // Setup digital pin 40 as an output
  pinMode(SEL2, OUTPUT);         // Setup digital pin 41 as an output   
  
  // Setup Emitter Pins for Optical Sensors 1-4
  pinMode(E1, OUTPUT);           // Setup digital pin 46 as an output
  pinMode(E2, OUTPUT);           // Setup digital pin 47 as an output
  pinMode(E3, OUTPUT);           // Setup digital pin 48 as an output
  pinMode(E4, OUTPUT);           // Setup digital pin 49 as an output
  
  // Setup Interrupt Service Routine driven by a rising edge on int 0(Pin2) for E-Stop
  pinMode(2, INPUT);                     // Setup digital pin 2 as an input
  attachInterrupt(0,STOP,RISING);        // Setup 'STOP' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a rising edge on int 1(Pin3) for Start Button
  pinMode(3, INPUT);                     // Setup digital pin 3 as an input
  attachInterrupt(1,START,RISING);       // Setup 'START' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a CHANGING edge on int 2(Pin21) for RECEIVER 4
  pinMode(21, INPUT);                     // Setup digital pin 21 as an input
  attachInterrupt(2,RECEIVER4,CHANGE);    // Setup 'RECEIVER1' interrupt to begin on a CHANGING edge 
 
  // Setup Interrupt Service Routine driven by a CHANGING edge on int 3(Pin20) for RECEIVER 3
  pinMode(20, INPUT);                     // Setup digital pin 20 as an input
  attachInterrupt(3,RECEIVER3,CHANGE);    // Setup 'RECEIVER2' interrupt to begin on a CHANGING edge  

  // Setup Interrupt Service Routine driven by a CHANGING edge on int 4(Pin19) for RECEIVER 2
  pinMode(19, INPUT);                     // Setup digital pin 19 as an input
  attachInterrupt(4,RECEIVER2,CHANGE);    // Setup 'RECEIVER3' interrupt to begin on a CHANGING edge
  
  // Setup Interrupt Service Routine driven by a CHANGING edge on int 5(Pin18) for RECEIVER 1
  pinMode(18, INPUT);                     // Setup digital pin 18 as an input
  attachInterrupt(5,RECEIVER1,CHANGE);    // Setup 'RECEIVER4' interrupt to begin on a CHANGING edge
  
  Stop_Motor();
  
  // Reset Encoder Counter by toggling RST pin
  digitalWrite(RST, LOW);          
  delay(100);
  digitalWrite(RST, HIGH); 
  
  // Adjust PWM Timers
  int myEraser = 7;           // Create eraser mask for bits 0,1 and 2
  TCCR0B &= ~myEraser;        // Clear last 3 bits
  TCCR0B |= 2;                // Set last 3 bits to 2 for PWM @ 7.8kHz
  TCCR3B &= ~myEraser;        // Clear last 3 bits
  TCCR3B |= 2;                // Set last 3 bits to 2 for PWM @ 4kHz
  
  // Setup Timer for Control Loop Frequency
  MsTimer2::set(Period, flash);   // Set Control Loop Frequency (Hz) to 1000/Period
  MsTimer2::start();              // Start timer
}

void loop()
{  
  // Define Local Variables
  int i = 0;                              // Sequence counter
  signed long int Encoder_Position = 0;   // encoder position   
  float Freq = 0;                         // Calculated Control Loop Frequency  
  float integral = 0;                     // Integral error term for PID Control 
  float error = 0;                        // Position error
  float old_error = 0;                    // Previous position error 
  float Controller_Output = 0;            // Control loop Controller_Output 
  byte Stop_bit = 0;                      // E-Stop input flag 
  unsigned long int Start_Count = 0;      // Start of sequence time count
  float Elapsed_Time = 0;                 // Total time for sequence 
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Local Variables 
  //////////////////////////////////////////////////////////////////////////////////////
  
  unsigned long int Current_Feedback_Total;  // accumulator variable for filtering Current Feedback
  unsigned int Current_Feedback;             // average value of Current Feedback    

  float Kd_Output = 0;               		 // Derivative output
  signed int PWM_Output = 0;          		 // Duty Cycle sent to Motor Controller
  signed long int EncoderA = 0;
  signed long int EncoderB = 0; 
  signed long int Target = 0; 

  //////////////////////////////////////////////////////////////////////////////////////
  
  // Reset Global Variables  
  go = 0;                             // Reset START flag      
  Stop = 0;                           // Reset E-STOP flag   
  Time_Count = 0;                     // Reset global Time Counter (in milliseconds)
  DONE = 0;
  
  Freq = 1000/float(Period);          // Calculate Control Loop Frequency
  KdxFreq = Kd*Freq;                  // Combined gain of Kd*Freq  
  
  // Turn off Emitters of Optical Sensors
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
  digitalWrite(E3, LOW);
  digitalWrite(E4, LOW);
  
  Stop_Motor();
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Setup Code
  //////////////////////////////////////////////////////////////////////////////////////
  
  VtoPWM = 255/V_in;                    // Conversion factor of Volts to PWM 
  //PeriodinSeconds = (float)Period/1000; // Control Loop Period converted to seconds from milliseconds

  //////////////////////////////////////////////////////////////////////////////////////
    
  while(!go);                    // Wait for START push button to be pressed 
  delay(500);                    // 500ms to debounce switch
  go = 0;                        // Reset START flag 
  
  Serial.println("Wack em All Started.");
  Serial.println("Good Luck...");

  // Ready motor
  digitalWrite(En_Out, HIGH);        // Set Motor Controller Enable pin HIGH
  digitalWrite(Disable_Out, HIGH);   // Set Motor Controller Disable pin HIGH
  
  Start_Count = Time_Count;
  
  for(i=0;i < TS_Length;i++)     
  {  
    if(Target_Sequence[i] == 1)
      digitalWrite(E1, HIGH);
    else if(Target_Sequence[i] == 2)
      digitalWrite(E2, HIGH);    
    else if(Target_Sequence[i] == 3)
      digitalWrite(E3, HIGH);
    else if(Target_Sequence[i] == 4)
      digitalWrite(E4, HIGH);
   
    //////////////////////////////////////////////////////////////////////////////////////
    //                   User Defined Code
    //////////////////////////////////////////////////////////////////////////////////////
	
  Received = 0;

    PWM_Output = 30;
    digitalWrite(PWM1_Out, LOW);         // Set PWM1_Out pin low
    analogWrite(PWM2_Out, PWM_Output);   // Set PWM2_Out to PWM signal

	while (Received == 0);
  EncoderA = Received_Position;
  
  
	while(DONE == 0)
	{
  
	  //while (timer_go == 0);
  EncoderB = Received_Position;

  Target = floor((EncoderA+EncoderB)/2);
  
		Encoder_Count = Read_Encoder();     				// Read in current encoder position   
		error = Target - Encoder_Count;       	// Calculate new error value
    
		Controller_Output = Kp*(float)error;

		Kd_Output = (float(error-old_error))*KdxFreq;
		Controller_Output += Kd_Output; 
	   
		Controller_Output = Controller_Output*VtoPWM;      	// Convert from volts to PWM duty cycle  
		old_error = error;                                 	// Save old error value

    Stop_bit = digitalRead(2);  
    if((!Stop) && (!Stop_bit))
    {
      PWM_Output = (int)Controller_Output;
      
      if(PWM_Output >= 0)
      {
        if(PWM_Output > 255)                 // Limit Controller_Output to 0 - 255
          PWM_Output = 255;
        digitalWrite(PWM1_Out, LOW);         // Set PWM1_Out pin low
        analogWrite(PWM2_Out, PWM_Output);   // Set PWM2_Out to PWM signal
      }
      else
      {    
        PWM_Output = -PWM_Output;            // Invert signal to keep positive
        if(PWM_Output > 255)                 // Limit Controller_Output to 0 - 255
          PWM_Output = 255;
        analogWrite(PWM1_Out, PWM_Output);   // Set PWM1_Out to PWM signal 
        digitalWrite(PWM2_Out, LOW);         // Set PWM2_Out pin low
      }  
    }  
    else
    {
      Stop_Motor();
    }
	
	}
	
    //////////////////////////////////////////////////////////////////////////////////////   
    //////////////////////////////////////////////////////////////////////////////////////     
        
    Received = 0;
    DONE = 0;    
        
    Serial.print("Target ");
    Serial.print(i); 
    Serial.println(" completed!"); 
    
    // Turn off Emitters of all Optical Sensors
    digitalWrite(E1, LOW);
    digitalWrite(E2, LOW);
    digitalWrite(E3, LOW);
    digitalWrite(E4, LOW);       
  }
  
  Elapsed_Time = (float)(Time_Count - Start_Count)/500;
  Serial.print("Elapsed Time = ");
  Serial.print(Elapsed_Time);
  Serial.println("seconds");
  Serial.println();
  
  Stop_Motor();

  delay(500);               // 500ms to debounce switch
} 

// Timer Sub-Routine for Control Loop
void flash()
{
  timer_go = 1;             // Set Timer flag
  Time_Count++;             // Increment Time_Count variable
  if(((Time_Count - Received_Count) >= 1000) && (Received == 1))   // Hold there for 2 seconds?
    DONE = 1;
}  

// Interrupt Service Rountine triggered by RISING edge from user activated
// momentary push button
void START()
{
    go = 1;                 // Set Push Button flag  
}   
       
// Interrupt Service Rountine triggered by RISING edge from E-Stop
void STOP()
{
  Stop = 1;                            // Set STOP flag

  Stop_Motor();
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 1
void RECEIVER1()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  
 
 
  Receiver_Bit = digitalRead(18);
  if(Receiver_Bit)
  {  
    Received = 1;
    Received_Count = Time_Count;
  }    
  else
    Received = 0;
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 2
void RECEIVER2()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  
 
  Receiver_Bit = digitalRead(19);
  if(Receiver_Bit)
  {  
    Received = 1;
    Received_Count = Time_Count;
  }    
  else
    Received = 0;
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 3
void RECEIVER3()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  
 
  Receiver_Bit = digitalRead(20);
  if(Receiver_Bit)
  {  
    Received = 1;
    Received_Count = Time_Count;
  }    
  else
    Received = 0;
}  

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 4
void RECEIVER4()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  
 
  Receiver_Bit = digitalRead(21);
  if(Receiver_Bit)
  {  
    Received = 1;
    Received_Count = Time_Count;
  }    
  else
    Received = 0;
}
