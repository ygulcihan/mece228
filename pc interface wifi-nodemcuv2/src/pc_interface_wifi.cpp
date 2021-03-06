#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <RadioHead.h>
#include <RH_ASK.h>

RH_ASK rf(2000, D0, D1, D2, true);
WiFiClient client;

const char *ssid = "cilgin robot 3.0";
const char *password = "12345678";

String rfMessage;
String lastRfMessage;
bool recieved = false;

int HTTP_PORT = 80;
String HTTP_METHOD = "GET";
char HOST_NAME[] = "192.168.4.1";

void ledOn();
void ledOff();
void blinkLed();
void stop();
void forward();
void left();
void right();
void reverse();
void lineFollow();
void serialRead();
void rfComm();
void rfTest();

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("power on");
  WiFi.begin(ssid, password);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  while (WiFi.status() != WL_CONNECTED)
  {
    blinkLed();
    delay(500);
    Serial.print(".");
  }

  if (client.connect(HOST_NAME, HTTP_PORT))
  {
    Serial.println();
    Serial.println("Connected to server");
  }

  if (!rf.init())
  {
    Serial.println("rf init failed");
  }
}

void loop()
{
  serialRead();
  rfComm();
  //rfTest();
}

void rfComm()
{
  uint8_t buf[3];
  uint8_t buflen = sizeof(buf);

  if (rf.recv(buf, &buflen))
  {
    String rfBuffer = String((char *)buf);
    recieved = true;

    for (int i = 0; i < rfBuffer.length(); i++)
    {
      if (rfBuffer.substring(i, i + 1) == "c")
      {
        rfMessage = rfBuffer.substring(i, i + 3);
        break;
      }
      else if (rfBuffer.substring(i, i + 1) == "s")
      {
        rfMessage = rfBuffer.substring(i, i + 3);
        break;
      }
    }

    bool a = false;

    if(lastRfMessage == "stp" && rfMessage == "stp")
    {
      a = true;
    }

    if(!a)
    {
    Serial.print(rfMessage);
    lastRfMessage = rfMessage;
    }
  }

  else
  {
    recieved = false;
  }
}

void serialRead()
{
  String readString;
  String Q;

  while (Serial.available())
  {
    delay(1);
    if (Serial.available() > 0)
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

  String chkSpd = Q.substring(0, 5);
  String spd = Q.substring(5, 6);

  if (Q == "stop")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/stop" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }

    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/stop" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "forward")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/forward" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "left")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/left" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "right")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/right" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "reverse")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/reverse" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "lineFollow")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      client.println(HTTP_METHOD + " " + "/lineFollow" + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
      blinkLed();
    }
  }

  else if (Q == "led on")
  {
    ledOn();
  }

  else if (Q == "led off")
  {
    ledOff();
  }

  if (chkSpd == "speed")
  {
    if (client.connect(HOST_NAME, HTTP_PORT))
    {
      // Serial.print("sent speed:");
      // Serial.println(spd);

      client.println(HTTP_METHOD + " " + "/speed" + spd + " HTTP/1.1");
      client.println("Host: " + String(HOST_NAME));
      client.println("Connection: close");
      client.println();
    }
  }
}

void ledOn()
{
  if (client.connect(HOST_NAME, HTTP_PORT))
  {
    client.println(HTTP_METHOD + " " + "/ledOn" + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println();
    blinkLed();
  }
}
void ledOff()
{
  if (client.connect(HOST_NAME, HTTP_PORT))
  {
    client.println(HTTP_METHOD + " " + "/ledOff" + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println();
    blinkLed();
  }
}

void rfTest()
{
  rfComm();

  if (recieved)
  {
    Serial.print("Message Recieved: ");
    Serial.print(rfMessage);
    Serial.println("");
  }
}

void blinkLed()
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
}
