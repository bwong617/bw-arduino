const int inPin = 2; //this pin receiving the signal to be measured
volatile int rising_edge_counter = 0; //stores the number of rising edges of the incoming signal (increamented by interrupt)
volatile int start_count = 0;
volatile int end_count = 0;
long counter_freq;
float period;
long startcount; //stores Timer0 count value at the first rising edge
long duration; // stores the difference in Timer0 counts between the first and second rising edges
float frequency; //stores the calculated frequency of the incoming signal

void setup()
{
  pinMode(inPin, INPUT);
  TCCR0B = (_BV(CS02) | _BV(CS00)); //Set Timer0 prescaling factor
  counter_freq = 16e6/1024; //Calculate actual counter frequency 16Mhz/prescale factor
  Serial.begin(9600);
  attachInterrupt(0, snowy, RISING); // interrupt 0 is mapped to pin 2 on the Uno, measure rising edges only
}

void loop()
{
  if (rising_edge_counter >= 11)
  {
    if (end_count > start_count)
    {
      duration = end_count - start_count;
      frequency= counter_freq/(float(duration));
      Serial.print(" Frequency is = ");
      Serial.println(frequency);
    }
    rising_edge_counter = 0;
  }
}
  /*Let the program count at least 11 cycles of the incoming signal (use rising_edge_counter to keep track) before you calculate frequency 
  ////Check to make sure that the second count measurement is bigger than the first (end_count vs. start_count), if it is
  calculate the number of Timer0 clocks between the first and second rising edges of the incoming signal
  ////calculate the period of the incoming signal (you have the number of clocks between rising edges and the frequency of Timer0)
  ////calculate the incoming signals frequency and print it.
  ////;reset the count (rising_edge_counter) of the incoming signal*/

void snowy()
{
    if (rising_edge_counter == 1)
    {
      start_count = TCNT0;
    }
    else if (rising_edge_counter == 2)
    {
      end_count = TCNT0;
    }
    rising_edge_counter++;
}
