#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"
#include <math.h>

String message = "";
bool messageReady = false;

const char *ssid = "cilgin robot 3.0";
const char *password = "12345678";

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

AsyncWebServer server(80);

String ledOn();
String ledOff();
void ultrasonicSensor();
void ultrasonicTest();
void sevenBit();

void setup()
{
  Serial.begin(115200);
  Serial.println("power on");
  Serial.println("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  server.on(
      "/ledOn", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", ledOn().c_str());
      });
  server.on(
      "/ledOff", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", ledOff().c_str());
      });

  server.begin();
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

String ledOn()
{
  
  return String("Led ON");
}

String ledOff()
{

  return String("Led OFF");
}
