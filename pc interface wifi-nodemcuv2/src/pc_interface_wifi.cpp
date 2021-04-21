#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>


WiFiClient client;

const char* ssid = "cilgin robot 3.0";
const char* password = "12345678";

int    HTTP_PORT   = 80;
String HTTP_METHOD = "GET";
char   HOST_NAME[] = "192.168.4.1";

void ledOn();
void ledOff();

void setup() 
{
Serial.begin(9600);
delay(3000);
Serial.println("power on");
WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) 
 {
    delay(500);
    Serial.print(".");
  }

 if(client.connect(HOST_NAME, HTTP_PORT)) 
 {
    Serial.println();
    Serial.println("Connected to server");
 }
}

void loop() 
{
String readString;
String Q;

while (Serial.available())
{
  delay(1);
  if(Serial.available()>0)
  {
    char c = Serial.read();
    if (isControl(c))
    {
      break;
    }
    readString += c;
  }
}


Q = readString;
if (Q=="led on")
{
  ledOn();
}

else if(Q=="led off")
{
  ledOff();
}

}

void ledOn()
{
  if (client.connect(HOST_NAME, HTTP_PORT)) 
  {
    Serial.println("send:led on");
    client.println(HTTP_METHOD + " " + "/ledOn" + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println();
  }
  else 
  {
    Serial.println("connection failed");
    Serial.println();
  }
 }
 void ledOff()
{
  if (client.connect(HOST_NAME, HTTP_PORT)) 
  {
    Serial.println("send:led off");
    client.println(HTTP_METHOD + " " + "/ledOff" + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println();
  }
  else 
  {
    Serial.println("connection failed");
    Serial.println();
  }
 }
