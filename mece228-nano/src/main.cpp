#include <Arduino.h>

int i = 319;

void setup() 
{
  Serial.begin(9600);

}

void loop() 
{
  Serial.print(i);
  delay(100);
}