#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial link(4, 5); // Rx, Tx

// Line Sensor Variables //
int readL0;
String colorL0;
int readL1;
String colorL1;
int readL2;
String colorL2;
int readL3;
String colorL3;
int readL4;
String colorL4;
int readL5;
String colorL5;
int readL6;
String colorL6;
int readL7;
String colorL7;

int readR0;
String colorR0;
int readR1;
String colorR1;
int readR2;
String colorR2;
int readR3;
String colorR3;
int readR4;
String colorR4;
int readR5;
String colorR5;
int readR6;
String colorR6;
int readR7;
String colorR7;

// Ultrasonic Sensor Variables //
#define echoPinL 7
#define trigPinL 6
unsigned long durationL;
unsigned long distanceL;

#define echoPinR 8
#define trigPinR 9
unsigned long durationR;
unsigned long distanceR;

#define echoPinB 10
#define trigPinB 11
unsigned long durationB;
unsigned long distanceB;

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
pinMode(A8, INPUT);
pinMode(A9, INPUT);
pinMode(A10, INPUT);
pinMode(A11, INPUT);
pinMode(A12, INPUT);
pinMode(A13, INPUT);
pinMode(A14, INPUT);
pinMode(A15, INPUT);

// Infrared Sensor Pins //
pinMode(irPin, INPUT);

// Ultrasonic Sensor Pins //
pinMode(echoPinL, INPUT);
pinMode(trigPinL, OUTPUT);

pinMode(echoPinR, INPUT);
pinMode(trigPinR, OUTPUT);

pinMode(echoPinB, INPUT);
pinMode(trigPinB, OUTPUT);
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

void ultrasonicSensorL()
{

digitalWrite(trigPinL,LOW);
delayMicroseconds(2);
digitalWrite(trigPinL, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinL, LOW);

durationL = pulseIn(echoPinL, HIGH);
distanceL = durationL / 58.2;

delay(50);

}

void ultrasonicSensorR()
{

digitalWrite(trigPinR,LOW);
delayMicroseconds(2);
digitalWrite(trigPinR, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinR, LOW);

durationR = pulseIn(echoPinR, HIGH);
distanceR = durationR / 58.2;

delay(50);

}

void ultrasonicSensorB()
{

digitalWrite(trigPinB,LOW);
delayMicroseconds(2);
digitalWrite(trigPinB, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinB, LOW);

durationB = pulseIn(echoPinB, HIGH);
distanceB = durationB / 58.2;

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
readL0 = analogRead(A0);
if (readL0<=800)
{
    colorL0 = "white";
}
else
{
    colorL0 = "black";
}

Serial.print("0: ");
Serial.print(readL0);
Serial.print(",");
Serial.print(colorL0);
Serial.print("  ");


readL1 = analogRead(A1);
if (readL1<=800)
{
    colorL1 = "white";
}
else
{
    colorL1 = "black";
}

Serial.print("1: ");
Serial.print(readL1);
Serial.print(",");
Serial.print(colorL1);
Serial.print("  ");


readL2 = analogRead(A2);
if (readL2<=800)
{
    colorL2 = "white";
}
else
{
    colorL2 = "black";
}

Serial.print("2: ");
Serial.print(readL2);
Serial.print(",");
Serial.print(colorL2);
Serial.print("  ");


readL3 = analogRead(A3);
if (readL3<=800)
{
    colorL3 = "white";
}
else
{
    colorL3 = "black";
}

Serial.print("3: ");
Serial.print(readL3);
Serial.print(",");
Serial.print(colorL3);
Serial.print("  ");


readL4 = analogRead(A4);
if (readL4<=800)
{
    colorL4 = "white";
}
else
{
    colorL4 = "black";
}

Serial.print("4: ");
Serial.print(readL4);
Serial.print(",");
Serial.print(colorL4);
Serial.print("  ");


readL5 = analogRead(A5);
if (readL5<=800)
{
    colorL5 = "white";
}
else
{
    colorL5 = "black";
}

Serial.print("5: ");
Serial.print(readL5);
Serial.print(",");
Serial.print(colorL5);
Serial.print("  ");


readL6 = analogRead(A6);
if (readL6<=800)
{
    colorL6 = "white";
}
else
{
    colorL6 = "black";
}

Serial.print("6: ");
Serial.print(readL6);
Serial.print(",");
Serial.print(colorL6);
Serial.print("  ");


readL7 = analogRead(A7);
if (readL7<=800)
{
    colorL7 = "white";
}
else
{
    colorL7 = "black";
}

Serial.print("7: ");
Serial.print(readL7);
Serial.print(",");
Serial.print(colorL7);
Serial.println("");


readR0 = analogRead(A8);
if (readR0<=800)
{
    colorR0 = "white";
}
else
{
    colorR0 = "black";
}

Serial.print("0: ");
Serial.print(readR0);
Serial.print(",");
Serial.print(colorR0);
Serial.print("  ");


readR1 = analogRead(A9);
if (readR1<=800)
{
    colorR1 = "white";
}
else
{
    colorR1 = "black";
}

Serial.print("1: ");
Serial.print(readR1);
Serial.print(",");
Serial.print(colorR1);
Serial.print("  ");


readR2 = analogRead(A10);
if (readR2<=800)
{
    colorR2 = "white";
}
else
{
    colorR2 = "black";
}

Serial.print("2: ");
Serial.print(readR2);
Serial.print(",");
Serial.print(colorR2);
Serial.print("  ");


readR3 = analogRead(A11);
if (readR3<=800)
{
    colorR3 = "white";
}
else
{
    colorR3 = "black";
}

Serial.print("3: ");
Serial.print(readR3);
Serial.print(",");
Serial.print(colorR3);
Serial.print("  ");


readR4 = analogRead(A12);
if (readR4<=800)
{
    colorR4 = "white";
}
else
{
    colorR4 = "black";
}

Serial.print("4: ");
Serial.print(readR4);
Serial.print(",");
Serial.print(colorR4);
Serial.print("  ");


readR5 = analogRead(A13);
if (readR5<=800)
{
    colorR5 = "white";
}
else
{
    colorR5 = "black";
}

Serial.print("5: ");
Serial.print(readR5);
Serial.print(",");
Serial.print(colorR5);
Serial.print("  ");


readR6 = analogRead(A14);
if (readR6<=800)
{
    colorR6 = "white";
}
else
{
    colorR6 = "black";
}

Serial.print("6: ");
Serial.print(readR6);
Serial.print(",");
Serial.print(colorR6);
Serial.print("  ");


readR7 = analogRead(A15);
if (readR7<=800)
{
    colorR7 = "white";
}
else
{
    colorR7 = "black";
}

Serial.print("7: ");
Serial.print(readR7);
Serial.print(",");
Serial.print(colorR7);
Serial.println("");

}

}

