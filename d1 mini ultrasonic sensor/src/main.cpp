#include <Arduino.h>
#include <math.h>
#include <ESP8266WiFi.h>

#define echoPinR D7
#define trigPinR D8
long durationR;
long distanceR;

#define b0 D0
#define b2 D1
#define b4 D2
#define b8 D3
#define b16 D4
#define b32 D5
#define b64 D6
int sentValue[7];

void ultrasonicSensor();
void ultrasonicTest();
void sevenBit();


void setup() 
{
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  pinMode(b0, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(b4, OUTPUT);
  pinMode(b8, OUTPUT);
  pinMode(b16, OUTPUT);
  pinMode(b32, OUTPUT);
  pinMode(b64, OUTPUT);
}

void loop() 
{
  //ultrasonicTest();
  sevenBit();
}


void ultrasonicTest()
{
  ultrasonicSensor();
  Serial.println(distanceR);
}

void sevenBit()
{
  ultrasonicSensor();

  if(distanceR > 1200)
  {
    distanceR = 0;
  }

  else if(distanceR > 254)
  {
    distanceR = 254;
  }

  else if (distanceR <= 3)
  {
    distanceR = 0;
  }

  int decNum = round(distanceR / 2);

  for (int i = 0; i < 7; i++)
  {
    sentValue[i] = decNum % 2;
    decNum = decNum / 2;
    //Serial.println(sentValue[i]);
  }

  digitalWrite(b0, sentValue[0]);
  digitalWrite(b2, sentValue[1]);
  digitalWrite(b4, sentValue[2]);
  digitalWrite(b8, sentValue[3]);
  digitalWrite(b16, sentValue[4]);
  digitalWrite(b32, sentValue[5]);
  digitalWrite(b64, sentValue[6]);
  
  Serial.println(distanceR);
}

void ultrasonicSensor()
{

  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH);

  distanceR = durationR / 58.2;
} 