#include "SD.h"
#define SD_ChipSelectPin 4
#include "TMRpcm.h"
#include "SPI.h"

int ledPin = A3;                // LED 
int pirPin = 2;                 // PIR Out pin 
int pirStat = 0;                   // PIR status

//Motor A
const int motorPin1  = A1;  // Pin 10 of L293
//hypersonic sensor pins
const int echoPin = A4;
const int trigPin = 7;

float duration, distance;

int red_light_pin = 6;
int green_light_pin = 5;
int blue_light_pin = 10;

int currentRed = random(255);
int currentGreen = random(255);
int currentBlue = random(255);

boolean redIncrease = true;
boolean blueIncrease = true;
boolean greenIncrease = true;

boolean playSound = false;


unsigned long starttime = millis();
unsigned long endtime = starttime;

int potPin = A0; // Potentiometer output connected to analog pin 3
int potVal = 0; // Variable to store the input from the potentiometer

TMRpcm tmrpcm;

//###############################################################3
void setup()
{
  //setup the speaker
  tmrpcm.speakerPin = 9;
  if (!SD.begin(SD_ChipSelectPin)) 
  {
    Serial.println("SD fail");
    return;
  }
  tmrpcm.setVolume(5);
  //tmrpcm.play("back.wav", 3000);
  
  //print the name of the song
  //Serial.print("Song: work.wav");

  //setup the infrared sensor as well as associated LED
  pinMode(ledPin, OUTPUT);     
  pinMode(pirPin, INPUT); 
  
  //set up hypersonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //set up motor
  pinMode(motorPin1, OUTPUT);

  //set up our rainbow light
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);

  
  Serial.begin(9600);
}
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void loop()
{ 
  potVal = analogRead(potPin);
  Serial.print(potVal);
  infraredSensor();
  hypersonicSensor();
  //lightupRainbow(currentRed, currentGreen, currentBlue, redIncrease, greenIncrease, blueIncrease);

  if(potVal > 700)
  {
    lightupRainbow(currentRed, currentGreen, currentBlue, redIncrease, greenIncrease, blueIncrease);
  }
  if(potVal < 400)
  {
    playSound = true;
  }
  else
  {
    playSound = false;
  }
  if(potVal < 100)
  {
    playSound = false;
    tmrpcm.play("sun.wav", 3000);
  }
  delay(100);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

//#####################################################################################################

void infraredSensor()
{
 pirStat = digitalRead(pirPin); 
 if (pirStat == HIGH) 
 {            
  // if motion detected
   digitalWrite(ledPin, HIGH);  // turn LED ON
   Serial.println("Hey I got you!!!");
   if(playSound = true)
   {
    tmrpcm.play("back.wav", 3000);
   }
 } 
 else 
 {
  //Serial.println("this didnt work");
  digitalWrite(ledPin, LOW); // turn LED OFF if we have no motion
 }
}
//####################################################################
int hypersonicSensor()
{
  //test the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //send out hypersonic signals
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //find how long it took for signal to return
  //basically how long the sensor is powered high from the returning signal
  duration = pulseIn(echoPin, HIGH);
  //use speed of sound and time to calculate distance
  distance = (duration*.0343)/2; 
  Serial.print("Distance: ");
  Serial.println(distance); 

  distance = (int)distance;
  if(distance * 2 < 255)
  {
    RGB_color(255-distance * 2, distance * 2, 0);
    if(distance < 20)
    {
      fullStop();
    }
    else
    {
      moveForward();
    }
  }
  else
  {
    RGB_color(0, 255, 0);
    digitalWrite(motorPin1, HIGH);
  }
  
  return (int)distance;
}

void moveForward()
{

  if((endtime - starttime) <= 500) // do this loop for up to 1000mS
  {   
    digitalWrite(motorPin1, HIGH);
    endtime = millis();
  } 
  else if((endtime - starttime) <= 1500)
  {
    digitalWrite(motorPin1, LOW);   
  }
  else
  {
    starttime = millis();
  }
}
//####################################################################333
void fullStop()
{
    digitalWrite(motorPin1, LOW);
}
//###########################################
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

void lightupRainbow(int red, int green, int blue, boolean rinc, boolean ginc, boolean binc)
{

  int redIterator = random(20);
  int greenIterator = random(20);
  int blueIterator = random(20);


  //logic to increase or decrease our colors

  if(rinc)
  {
    currentRed = currentRed + redIterator;
  }
  else
  {
    currentRed = currentRed - redIterator;
  }
  if(ginc)
  {
    currentGreen = currentGreen + greenIterator;
  }
  else
  {
    currentGreen = currentGreen - greenIterator ;
  }
  if(binc)
  {
    currentBlue = currentBlue + blueIterator;
  }
  else
  {
    currentBlue = currentBlue - blueIterator;
  }

  //check if any of our variables have exceeded allowed values
  if(currentRed >= 255)
  {
    currentRed = 255;
    redIncrease = false;
  }
  if(currentRed <= 0)
  {
    currentRed = 0;
    redIncrease = true;
  }
  if(currentGreen >= 255)
  {
    currentGreen = 255;
    greenIncrease = false;
  }
  if(currentGreen <= 0)
  {
    currentGreen = 0;
    greenIncrease = true;
  }
  if(currentBlue >= 255)
  {
    currentBlue = 255;
    blueIncrease = false;
  }
  if(currentBlue <= 0)
  {
    currentBlue = 0;
    blueIncrease = true;
  }

  Serial.print("colors: ");
  Serial.print(currentRed);
  Serial.print(" ");
  Serial.print(currentGreen);
  Serial.print(" ");
  Serial.print(currentBlue);
  Serial.println();
  Serial.println(redIncrease);
  RGB_color(currentRed, currentGreen, currentBlue);
}
