#include <Arduino.h>
#include <math.h>

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

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
}

void loop() 
{
  ultrasonicSensor();
  ultrasonicTest();
  sevenBit();
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

void ultrasonicTest()
{
  Serial.println(distanceR);
}

void sevenBit()
{
  int decNum = round(distanceR / 8);

  for (int i = 0; i < 7; i++)
  {
    sentValue[i] = decNum % 2;
    decNum = decNum / 2;
    //Serial.println(sentValue[i]);
  }

  if(1)
  {

  }
}
