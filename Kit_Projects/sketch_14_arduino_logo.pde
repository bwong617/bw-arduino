import processing.serial.*;

//initialize Serial object
Serial myPort;

//create object to hold image
PImage logo;

//background hue of Arduino logo
int bgcolor = 0;

void setup(){
  
  //HSB: Hue, Saturation, Brightness; 0-255
  colorMode(HSB, 255);
  
  logo = loadImage("http://arduino.cc/logo.png");
  
  //display window size - scale to logo size
  //OLD//size(logo.width, logo.height);
  surface.setSize(logo.width, logo.height);
  
  println("Available serial ports:");
  //OLD//println(Serial.list());
  printArray(Serial.list());
  
  myPort = new Serial(this, Serial.list()[0], 9600);
}

void draw(){
  if(myPort.available() > 0){
    bgcolor = myPort.read();
    println(bgcolor);
  }
  background(bgcolor, 255, 255);
  image(logo, 0, 0);
}