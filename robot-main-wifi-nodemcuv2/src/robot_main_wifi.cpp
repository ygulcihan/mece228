#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"

const char* ssid = "cilgin robot 3.0";
const char* password = "12345678";
int ledPin = D1;

AsyncWebServer server(80);

void setup() 
{
Serial.begin(115200);
Serial.println("power on");
Serial.println("Setting AP (Access Point)…");
WiFi.softAP(ssid, password);
IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);

pinMode(LED_BUILTIN, OUTPUT); pinMode(ledPin, OUTPUT);

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
