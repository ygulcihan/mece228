#include <Arduino.h>
#include <ArduinoJson.h>


// Headlight Variables //
#define hlGND 7
#define hlR 5
#define hlG 6
#define hlB 4

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
#define echoPinL 24
#define trigPinL 22
long durationL;
long distanceL;

String message;
long distanceR;

// Infrared Sensor Variables //
#define irPin 12
bool objectDetected = false;

// Motor Driver Variables //
#define enableLF 13
#define inputLF1 14
#define inputLF2 15

#define enableLB 16
#define inputLB1 17
#define inputLB2 18

#define enableRF 19
#define inputRF1 20
#define inputRF2 21

#define enableRB 50
#define inputRB1 51
#define inputRB2 52

// Function Declarations //
void lineSensor();
void serialRead();
void ultrasonicSensors();
void infraredSensor();
void infraredTest();
void lineTest();
void ultrasonicTest();
void headlightRed();
void jsonComm();

void setup() 
{

Serial.begin(9600);


// Headlight Pins //
pinMode(hlGND, INPUT);
pinMode(hlR, OUTPUT);
pinMode(hlG, OUTPUT);
pinMode(hlB, OUTPUT);

digitalWrite(hlGND, LOW);

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

// Motor Driver Pins //
pinMode(enableLF, OUTPUT);
pinMode(inputLF1, OUTPUT);
pinMode(inputLF2, OUTPUT);

pinMode(enableLB, OUTPUT);
pinMode(inputLB1,OUTPUT);
pinMode(inputLB2,OUTPUT);

pinMode(enableRF, OUTPUT);
pinMode(inputRF1, OUTPUT);
pinMode(inputRF2, OUTPUT);

pinMode(enableRB, OUTPUT);
pinMode(inputRB1, OUTPUT);
pinMode(inputRB2, OUTPUT);

}

// start of loop //

void loop() 
{

jsonComm();
lineSensor();
serialRead();
infraredSensor();
ultrasonicSensors();
headlightRed();

if(distanceR != 0)
{   
    pinMode(53, OUTPUT);
    digitalWrite(53, HIGH);
}
else
{   
    pinMode(53, OUTPUT);
    digitalWrite(53, LOW);
}

}

// end of loop //

void jsonComm()
{

}

void infraredSensor()
{
objectDetected = digitalRead(irPin);
}

void infraredTest()
{
    if(objectDetected)
    {
        Serial.println("object detected");
    }
    else
    {
        Serial.println("no object");
    }
    delay(200);
}

void ultrasonicSensors()
{

digitalWrite(trigPinL,LOW);
delayMicroseconds(2);

digitalWrite(trigPinL,HIGH);
delayMicroseconds(10);

digitalWrite(trigPinL,LOW);
durationL = pulseIn(echoPinL, HIGH);

distanceL = durationL / 58.2;

}

void ultrasonicTest()
{
    Serial.print("distanceL: ");
    Serial.print(distanceL);
    Serial.print("  ");
    Serial.print("distanceR: ");
    Serial.print(distanceR);
    Serial.println("  ");
}

void headlightRed()
{
    digitalWrite(hlB, HIGH);
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

while (Q=="linetest on")
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
    if (Q=="linetest off")
    {
        break;
        Serial.flush();
    }
lineSensor();
lineTest();
}
 
while (Q=="ultrasonictest on")
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
    if (Q=="ultrasonictest off")
    {
        break;
        Serial.flush();
    }
ultrasonicSensors();
jsonComm();
ultrasonicTest();
}

while (Q=="infraredtest on")
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
    if (Q=="infraredtest off")
    {
        break;
        Serial.flush();
    }
infraredTest();
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

readL1 = analogRead(A1);
if (readL1<=800)
{
    colorL1 = "white";
}
else
{
    colorL1 = "black";
}

readL2 = analogRead(A2);
if (readL2<=800)
{
    colorL2 = "white";
}
else
{
    colorL2 = "black";
}

readL3 = analogRead(A3);
if (readL3<=800)
{
    colorL3 = "white";
}
else
{
    colorL3 = "black";
}

readL4 = analogRead(A4);
if (readL4<=800)
{
    colorL4 = "white";
}
else
{
    colorL4 = "black";
}

readL5 = analogRead(A5);
if (readL5<=800)
{
    colorL5 = "white";
}
else
{
    colorL5 = "black";
}


readL6 = analogRead(A6);
if (readL6<=800)
{
    colorL6 = "white";
}
else
{
    colorL6 = "black";
}


readL7 = analogRead(A7);
if (readL7<=800)
{
    colorL7 = "white";
}
else
{
    colorL7 = "black";
}

readR0 = analogRead(A8);
if (readR0<=800)
{
    colorR0 = "white";
}
else
{
    colorR0 = "black";
}

readR1 = analogRead(A9);
if (readR1<=800)
{
    colorR1 = "white";
}
else
{
    colorR1 = "black";
}


readR2 = analogRead(A10);
if (readR2<=800)
{
    colorR2 = "white";
}
else
{
    colorR2 = "black";
}


readR3 = analogRead(A11);
if (readR3<=800)
{
    colorR3 = "white";
}
else
{
    colorR3 = "black";
}

readR4 = analogRead(A12);
if (readR4<=800)
{
    colorR4 = "white";
}
else
{
    colorR4 = "black";
}

readR5 = analogRead(A13);
if (readR5<=800)
{
    colorR5 = "white";
}
else
{
    colorR5 = "black";
}

readR6 = analogRead(A14);
if (readR6<=800)
{
    colorR6 = "white";
}
else
{
    colorR6 = "black";
}

readR7 = analogRead(A15);
if (readR7<=800)
{
    colorR7 = "white";
}
else
{
    colorR7 = "black";
}

}

void lineTest()
{
Serial.print("L0: ");
Serial.print(readL0);
Serial.print(",");
Serial.print(colorL0);
Serial.println("");

Serial.print("L1: ");
Serial.print(readL1);
Serial.print(",");
Serial.print(colorL1);
Serial.println("");

Serial.print("L2: ");
Serial.print(readL2);
Serial.print(",");
Serial.print(colorL2);
Serial.println("");

Serial.print("L3: ");
Serial.print(readL3);
Serial.print(",");
Serial.print(colorL3);
Serial.println("");

Serial.print("L4: ");
Serial.print(readL4);
Serial.print(",");
Serial.print(colorL4);
Serial.println("");

Serial.print("L5: ");
Serial.print(readL5);
Serial.print(",");
Serial.print(colorL5);
Serial.println("");

Serial.print("L6: ");
Serial.print(readL6);
Serial.print(",");
Serial.print(colorL6);
Serial.println("");

Serial.print("L7: ");
Serial.print(readL7);
Serial.print(",");
Serial.print(colorL7);
Serial.println("");

Serial.print("R0: ");
Serial.print(readR0);
Serial.print(",");
Serial.print(colorR0);
Serial.println("");

Serial.print("R1: ");
Serial.print(readR1);
Serial.print(",");
Serial.print(colorR1);
Serial.println("");

Serial.print("R2: ");
Serial.print(readR2);
Serial.print(",");
Serial.print(colorR2);
Serial.println("");

Serial.print("R3: ");
Serial.print(readR3);
Serial.print(",");
Serial.print(colorR3);
Serial.println("");

Serial.print("R4: ");
Serial.print(readR4);
Serial.print(",");
Serial.print(colorR4);
Serial.println("");

Serial.print("R5: ");
Serial.print(readR5);
Serial.print(",");
Serial.print(colorR5);
Serial.println("");

Serial.print("R6: ");
Serial.print(readR6);
Serial.print(",");
Serial.print(colorR6);
Serial.println("");

Serial.print("R7: ");
Serial.print(readR7);
Serial.print(",");
Serial.print(colorR7);
Serial.println("");
}

