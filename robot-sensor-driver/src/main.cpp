#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <MFRC522.h>

// Line Sensor Variables //
int readL0, readL1, readL2, readL3, readL4, readL5, readL6, readL7;
String colorL0, colorL1, colorL2, colorL3, colorL4, colorL5, colorL6, colorL7;

int readR0, readR1, readR2, readR3, readR4, readR5, readR6, readR7;
String colorR0, colorR1, colorR2, colorR3, colorR4, colorR5, colorR6, colorR7;

// Communication Pins & Variables //

/* 7-bit */
#define b0 28
#define b2 30
#define b4 32
#define b8 34
#define b16 36
#define b32 38
#define b64 40
int readValue[7];

/* Nodemcu */
#define obstComm 51
#define speedb1 49
#define speedb0 47
#define cpb1 45
#define cpb0 43


// Ultrasonic Sensor Variables //
#define echoPinL 22
#define trigPinL 24
long durationL, distanceL, distanceR;

// RFID Reader Variables//
#define SS_PIN 53
#define RST_PIN 19
MFRC522 mfrc522(SS_PIN, RST_PIN);
int checkpoint;
unsigned long dl1p;
unsigned long dl2p;
unsigned long dl3p;

// Infrared Sensor Variables //
#define irPin 12
bool objectDetected = false;

// Motor Driver Variables //
#define enableLF 10
#define inputLF1 35
#define inputLF2 31

#define enableLB 12
#define inputLB1 37
#define inputLB2 33

#define enableRF 8
#define inputRF1 23
#define inputRF2 29

#define enableRB 11
#define inputRB1 25
#define inputRB2 27

int speed;

// Obst Sensor //
#define obstSensor 39

// Function Declarations //
void lineSensor();
void serialRead();
void ultrasonicSensor();
void infraredSensor();
void infraredTest();
void lineTest();
void ultrasonicTest();
int sevenBitComm();
void rgbCalibrate();
void rfid();
void rfidTest();
void motorDev();
void nodemcu();

void forward();
void reverse();
void left();
void right();
void stop();

// start of setup //
void setup()
{
    Serial.begin(9600);
    SPI.begin();
    mfrc522.PCD_Init();

    // Communication Pins //
    /* 7bit */
    pinMode(b0, INPUT);
    pinMode(b2, INPUT);
    pinMode(b4, INPUT);
    pinMode(b8, INPUT);
    pinMode(b16, INPUT);
    pinMode(b32, INPUT);
    pinMode(b64, INPUT);
    /* nodemcu */
    pinMode(speedb0, INPUT);
    pinMode(speedb1, INPUT);
    pinMode(obstComm, OUTPUT);
    pinMode(cpb1, OUTPUT);
    pinMode(cpb0, OUTPUT);

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
    pinMode(inputLB1, OUTPUT);
    pinMode(inputLB2, OUTPUT);

    pinMode(enableRF, OUTPUT);
    pinMode(inputRF1, OUTPUT);
    pinMode(inputRF2, OUTPUT);

    pinMode(enableRB, OUTPUT);
    pinMode(inputRB1, OUTPUT);
    pinMode(inputRB2, OUTPUT);
}

// end of setup //

// start of loop //

void loop()
{
    sevenBitComm();
    nodemcu();
    ultrasonicTest();
    lineSensor();
    //serialRead();
    infraredSensor();
    ultrasonicSensor();
    rfid();
    //motorDev();
}
// end of loop //

int sevenBitComm()
{
    readValue[0] = digitalRead(b0);
    readValue[1] = digitalRead(b2);
    readValue[2] = digitalRead(b4);
    readValue[3] = digitalRead(b8);
    readValue[4] = digitalRead(b16);
    readValue[5] = digitalRead(b32);
    readValue[6] = digitalRead(b64);

    int recievedValue = readValue[0] + readValue[1] * 2 + readValue[2] * 4 + readValue[3] * 8 + readValue[4] * 16 + readValue[5] * 32 + readValue[6] * 64;
    distanceR = recievedValue * 2;

    return distanceR;
}


void nodemcu()
{
    speed = (digitalRead(speedb0) + 2*digitalRead(speedb1))*85;

    switch (checkpoint)
    {
    case 1:
        digitalWrite(cpb0, HIGH);
        digitalWrite(cpb1, LOW);
        break;

    case 2:
        digitalWrite(cpb0, LOW);
        digitalWrite(cpb1, HIGH);
        break;

    case 3:
        digitalWrite(cpb0, HIGH);
        digitalWrite(cpb1, HIGH);
        break;

    default:
        digitalWrite(cpb0, LOW);
        digitalWrite(cpb1, LOW);
        break;
    }
}

void rfid()
{
    // Look for new cards
    if (!mfrc522.PICC_IsNewCardPresent())
    {
        return;
    }
    // Select one of the cards
    if (!mfrc522.PICC_ReadCardSerial())
    {
        return;
    }

    String content = "";

    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
        content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
        content.concat(String(mfrc522.uid.uidByte[i], HEX));
    }

    content.toUpperCase();
    if (content.substring(1) == "AA 7A E1 80")
    {
        checkpoint = 1;

        unsigned long dly1 = millis();
        if (dly1 - dl1p >= 3000)
        {
            dl1p = dly1;
        }
    }

    else if (content.substring(1) == "79 CB 2F 5A")
    {
        checkpoint = 2;

        unsigned long dly2 = millis();
        if (dly2 - dl2p >= 3000)
        {
            dl2p = dly2;
        }
    }

    else if (content.substring(1) == "59 9D A4 5A")
    {
        checkpoint = 3;

        unsigned long dly3 = millis();
        if (dly3 - dl3p >= 3000)
        {
            dl3p = dly3;
        }
    }

    else
    {
        Serial.println(" Access denied");
    }
}

void infraredSensor()
{
    objectDetected = digitalRead(irPin);
}

void infraredTest()
{
    if (objectDetected)
    {
        Serial.println("object detected");
    }
    else
    {
        Serial.println("no object");
    }
    delay(200);
}

void ultrasonicSensor()
{

    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPinL, LOW);
    durationL = pulseIn(echoPinL, HIGH);

    distanceL = durationL / 58.2;
}

void ultrasonicTest()
{
    sevenBitComm();
    Serial.print("distanceL: ");
    Serial.print(distanceL);
    Serial.print("  ");
    Serial.print("distanceR: ");
    Serial.print(distanceR);
    Serial.println("  ");
}

// Movement Commands //
void forward()
{
    analogWrite(enableLF, speed);
    analogWrite(enableRF, speed);
    analogWrite(enableLB, speed);
    analogWrite(enableRB, speed);

    digitalWrite(inputLB1, HIGH);
    digitalWrite(inputLF1, HIGH);
    digitalWrite(inputRB1, HIGH);
    digitalWrite(inputRF1, HIGH);

    digitalWrite(inputRF2, LOW);
    digitalWrite(inputRB2, LOW);
    digitalWrite(inputLF2, LOW);
    digitalWrite(inputLB2, LOW);
}

void reverse()
{
    analogWrite(enableLF, speed);
    analogWrite(enableRF, speed);
    analogWrite(enableLB, speed);
    analogWrite(enableRB, speed);

    digitalWrite(inputLB1, LOW);
    digitalWrite(inputLF1, LOW);
    digitalWrite(inputRB1, LOW);
    digitalWrite(inputRF1, LOW);

    digitalWrite(inputRF2, HIGH);
    digitalWrite(inputRB2, HIGH);
    digitalWrite(inputLF2, HIGH);
    digitalWrite(inputLB2, HIGH);
}

void left()
{
    analogWrite(enableLF, 0);
    analogWrite(enableRF, speed);
    analogWrite(enableLB, 0);
    analogWrite(enableRB, speed);

    digitalWrite(inputLB1, LOW);
    digitalWrite(inputLF1, LOW);
    digitalWrite(inputRB1, HIGH);
    digitalWrite(inputRF1, HIGH);

    digitalWrite(inputRF2, LOW);
    digitalWrite(inputRB2, LOW);
    digitalWrite(inputLF2, LOW);
    digitalWrite(inputLB2, LOW);
}

void right()
{
    analogWrite(enableLF, speed);
    analogWrite(enableRF, 0);
    analogWrite(enableLB, speed);
    analogWrite(enableRB, 0);

    digitalWrite(inputLB1, LOW);
    digitalWrite(inputLF1, HIGH);
    digitalWrite(inputRB1, LOW);
    digitalWrite(inputRF1, HIGH);

    digitalWrite(inputRF2, LOW);
    digitalWrite(inputRB2, LOW);
    digitalWrite(inputLF2, LOW);
    digitalWrite(inputLB2, LOW);
}

void stop()
{
    analogWrite(enableLF, 0);
    analogWrite(enableRF, 0);
    analogWrite(enableLB, 0);
    analogWrite(enableRB, 0);

    digitalWrite(inputLB1, LOW);
    digitalWrite(inputLF1, LOW);
    digitalWrite(inputRB1, LOW);
    digitalWrite(inputRF1, LOW);

    digitalWrite(inputRF2, LOW);
    digitalWrite(inputRB2, LOW);
    digitalWrite(inputLF2, LOW);
    digitalWrite(inputLB2, LOW);
}

void serialRead()
{

    String readString;
    String Q;

    Q = readString;

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

    while (Q == "linetest on")
    {
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
            Q = readString;
        }
        if (Q == "linetest off")
        {
            break;
            Serial.flush();
        }
        lineSensor();
        lineTest();
    }

    while (Q == "ultrasonictest on")
    {
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
            Q = readString;
        }
        if (Q == "ultrasonictest off")
        {
            break;
            Serial.flush();
        }
        ultrasonicSensor();
        ultrasonicTest();
    }

    while (Q == "infraredtest on")
    {
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
            Q = readString;
        }
        if (Q == "infraredtest off")
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
    if (readL0 <= 800)
    {
        colorL0 = "white";
    }
    else
    {
        colorL0 = "black";
    }

    readL1 = analogRead(A1);
    if (readL1 <= 800)
    {
        colorL1 = "white";
    }
    else
    {
        colorL1 = "black";
    }

    readL2 = analogRead(A2);
    if (readL2 <= 800)
    {
        colorL2 = "white";
    }
    else
    {
        colorL2 = "black";
    }

    readL3 = analogRead(A3);
    if (readL3 <= 800)
    {
        colorL3 = "white";
    }
    else
    {
        colorL3 = "black";
    }

    readL4 = analogRead(A4);
    if (readL4 <= 800)
    {
        colorL4 = "white";
    }
    else
    {
        colorL4 = "black";
    }

    readL5 = analogRead(A5);
    if (readL5 <= 800)
    {
        colorL5 = "white";
    }
    else
    {
        colorL5 = "black";
    }

    readL6 = analogRead(A6);
    if (readL6 <= 800)
    {
        colorL6 = "white";
    }
    else
    {
        colorL6 = "black";
    }

    readL7 = analogRead(A7);
    if (readL7 <= 800)
    {
        colorL7 = "white";
    }
    else
    {
        colorL7 = "black";
    }

    readR0 = analogRead(A8);
    if (readR0 <= 800)
    {
        colorR0 = "white";
    }
    else
    {
        colorR0 = "black";
    }

    readR1 = analogRead(A9);
    if (readR1 <= 800)
    {
        colorR1 = "white";
    }
    else
    {
        colorR1 = "black";
    }

    readR2 = analogRead(A10);
    if (readR2 <= 800)
    {
        colorR2 = "white";
    }
    else
    {
        colorR2 = "black";
    }

    readR3 = analogRead(A11);
    if (readR3 <= 800)
    {
        colorR3 = "white";
    }
    else
    {
        colorR3 = "black";
    }

    readR4 = analogRead(A12);
    if (readR4 <= 800)
    {
        colorR4 = "white";
    }
    else
    {
        colorR4 = "black";
    }

    readR5 = analogRead(A13);
    if (readR5 <= 800)
    {
        colorR5 = "white";
    }
    else
    {
        colorR5 = "black";
    }

    readR6 = analogRead(A14);
    if (readR6 <= 800)
    {
        colorR6 = "white";
    }
    else
    {
        colorR6 = "black";
    }

    readR7 = analogRead(A15);
    if (readR7 <= 800)
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

void motorDev()
{

    digitalWrite(enableRF, LOW);
    digitalWrite(enableLF, LOW);
    digitalWrite(enableRB, LOW);

    digitalWrite(enableLB, HIGH);
    digitalWrite(inputLB1, HIGH);
    digitalWrite(inputLB2, LOW);

    delay(2000);

    digitalWrite(enableLB, HIGH);
    digitalWrite(inputLB1, LOW);
    digitalWrite(inputLB2, HIGH);

    delay(2000);

    /*
   digitalWrite(8, HIGH);
   digitalWrite(9, HIGH);
   digitalWrite(10, HIGH);
   digitalWrite(11, HIGH);
   digitalWrite(12, HIGH);
   digitalWrite(13, HIGH);
   */
}