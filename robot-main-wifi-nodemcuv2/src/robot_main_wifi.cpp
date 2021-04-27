#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"
#include <SoftwareSerial.h>

const char* ssid = "cilgin robot 3.0";
const char* password = "12345678";
int ledPin = D1;
#define echoPinR D7
#define trigPinR D8
long durationR;
long distanceR;

SoftwareSerial link(2,1); // Rx,Tx
AsyncWebServer server(80);

String ledOn();
String ledOff();
void ultrasonicSensor();

void setup() 
{
Serial.begin(115200);
Serial.println("power on");
Serial.println("Setting AP (Access Point)…");
WiFi.softAP(ssid, password);
IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);

link.begin(9600);

pinMode(LED_BUILTIN, OUTPUT); pinMode(ledPin, OUTPUT);
pinMode(trigPinR, OUTPUT);
pinMode(echoPinR, INPUT);

 server.on(
  "/ledOn", HTTP_GET, [](AsyncWebServerRequest *request)
 {
  request->send_P(200, "text/plain", ledOn().c_str());
 });
 server.on(
  "/ledOff", HTTP_GET, [](AsyncWebServerRequest *request)
 {
  request->send_P(200, "text/plain", ledOff().c_str());
 });

 server.begin();
}

void loop() 
{

ultrasonicSensor();

}

void ultrasonicSensor()
{

digitalWrite(trigPinR,LOW);
delayMicroseconds(2);

digitalWrite(trigPinR,HIGH);
delayMicroseconds(10);

digitalWrite(trigPinR,LOW);
durationR = pulseIn(echoPinR, HIGH);

distanceR = durationR / 58.2;

Serial.println(distanceR);
char send = distanceR;
link.write(send);
delay(50);

}

String ledOn()
{
  digitalWrite(ledPin,HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  return String("Led ON");  
}

String ledOff()
{
  digitalWrite(ledPin,LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  return String("Led OFF");
}
