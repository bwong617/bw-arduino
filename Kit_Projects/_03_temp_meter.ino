const int sensorPin = A0;
const float baselineTemp = 20.0;

void setup() {
  //open serial port; baud rate = 9600
  Serial.begin(9600);
  
  //initialize output pins
  for (int i = 2; i <= 4; i++){
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);  
  }
}

void loop() {

  //retrieve ADC value (0-1023) from temperature sensor
  int sensorVal = analogRead(sensorPin);
  //convert the ADC reading to pin voltage
  float voltage = (sensorVal / 1024.0) * 5.0;
  //convert voltage to temperature (degC)
  float temperature = (voltage - 0.5) * 100;
  
  Serial.print("Sensor (ADC): ");
  Serial.print(sensorVal);
  Serial.print(", Voltage (V): ");
  Serial.print(voltage);
  Serial.print(", Temp (degC): ");
  Serial.println(temperature);

  if(temperature < baselineTemp){
    for (int i = 2; i <= 4; i++)
      digitalWrite(i,LOW);  
  }
  else if(temperature >= baselineTemp+2 && temperature < baselineTemp+4){
    digitalWrite(2,HIGH);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
  }
  else if(temperature >= baselineTemp+4 && temperature < baselineTemp+6){
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,LOW);
  }
  else{
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
  }
  delay(1);
  
}
