#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial link(4, 5); // Rx, Tx

// Line Sensor Variables //
int read0;
String color0;
int read1;
String color1;
int read2;
String color2;
int read3;
String color3;
int read4;
String color4;
int read5;
String color5;
int read6;
String color6;
int read7;
String color7;

// Ultrasonic Sensor Variables //
#define echoPin1 6
#define trigPin1 7
unsigned long duration1;
unsigned long distance1;

#define echoPin2 8
#define trigPin2 9
unsigned long duration2;
unsigned long distance2;

// Infrared Sensor Variables //
#define irPin 10
bool objectDetected = false;

// Function Definitions //
void lineSensor();
void serialRead();
void ultrasonicSensor1();
void ultrasonicSensor2();
void infraredSensor();


void setup() 
{

Serial.begin(9600);
link.begin(9600);

// Line Sensor Pins //
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
pinMode(A4, INPUT);
pinMode(A5, INPUT);
pinMode(A6, INPUT);
pinMode(A7, INPUT);

// Infrared Sensor Pins //
pinMode(irPin, INPUT);

}

void loop() 
{

serialRead();
infraredSensor();
ultrasonicSensor1();
ultrasonicSensor2();

}

// end of loop //

void infraredSensor()
{

objectDetected = digitalRead(irPin);
Serial.println(objectDetected);

}

void ultrasonicSensor1()
{

digitalWrite(trigPin1,LOW);
delayMicroseconds(2);
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);

duration1 = pulseIn(echoPin1, HIGH);
distance1 = duration1 / 58.2;

delay(50);

}

void ultrasonicSensor2()
{

digitalWrite(trigPin2,LOW);
delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);

duration2 = pulseIn(echoPin2, HIGH);
distance2 = duration2 / 58.2;

delay(50);

}

void serialRead()
{

String readString;
String Q;

Q = readString;

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

while (Q=="on")
{
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
  Q = readString;
}
    if (Q=="off")
    {
        break;
        Serial.flush();
    }
lineSensor();
}
 
}

void lineSensor()
{
delay(50);
read0 = analogRead(A0);
if (read0<=800)
{
    color0 = "white";
}
else
{
    color0 = "black";
}

Serial.print("0: ");
Serial.print(read0);
Serial.print(",");
Serial.print(color0);
Serial.print("  ");


read1 = analogRead(A1);
if (read1<=800)
{
    color1 = "white";
}
else
{
    color1 = "black";
}

Serial.print("1: ");
Serial.print(read1);
Serial.print(",");
Serial.print(color1);
Serial.print("  ");


read2 = analogRead(A2);
if (read2<=800)
{
    color2 = "white";
}
else
{
    color2 = "black";
}

Serial.print("2: ");
Serial.print(read2);
Serial.print(",");
Serial.print(color2);
Serial.print("  ");


read3 = analogRead(A3);
if (read3<=800)
{
    color3 = "white";
}
else
{
    color3 = "black";
}

Serial.print("3: ");
Serial.print(read3);
Serial.print(",");
Serial.print(color3);
Serial.print("  ");


read4 = analogRead(A4);
if (read4<=800)
{
    color4 = "white";
}
else
{
    color4 = "black";
}

Serial.print("4: ");
Serial.print(read4);
Serial.print(",");
Serial.print(color4);
Serial.print("  ");


read5 = analogRead(A5);
if (read5<=800)
{
    color5 = "white";
}
else
{
    color5 = "black";
}

Serial.print("5: ");
Serial.print(read5);
Serial.print(",");
Serial.print(color5);
Serial.print("  ");


read6 = analogRead(A6);
if (read6<=800)
{
    color6 = "white";
}
else
{
    color6 = "black";
}

Serial.print("6: ");
Serial.print(read6);
Serial.print(",");
Serial.print(color6);
Serial.print("  ");


read7 = analogRead(A7);
if (read7<=800)
{
    color7 = "white";
}
else
{
    color7 = "black";
}

Serial.print("7: ");
Serial.print(read7);
Serial.print(",");
Serial.print(color7);
Serial.println("");

}

