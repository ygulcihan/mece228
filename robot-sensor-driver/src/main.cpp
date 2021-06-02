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
#define speedb1 25
#define speedb0 27
#define cpb1 2
#define cpb0 3
#define go1 22
#define go2 23
#define go3 24
unsigned int speed = 255;
unsigned int go = 0;

// RFID Reader Variables//
#define SS_PIN 53
#define RST_PIN 19
MFRC522 mfrc522(SS_PIN, RST_PIN);
int checkpoint;
unsigned long dl1p;
unsigned long dl2p;
unsigned long dl3p;

// Infrared Sensor Variables //
#define irPin 6
bool objectDetected = false;

// Motor Driver Variables //
#define enableL 8
#define enableR 9
#define dirLF 10
#define dirLR 11
#define dirRF 12
#define dirRR 13

// Obst Sensor //
#define obstSensor 39

// Function Declarations //
void lineSensor();
void serialRead();
void ultrasonicSensor();
void infraredSensor();
void infraredTest();
void lineTest();
void rfid();
void rfidTest();
void motorDev();
void comm();
void commTest();
void motors();

void lineFollow();
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

    pinMode(speedb0, INPUT);
    pinMode(speedb1, INPUT);
    pinMode(cpb0, OUTPUT);
    pinMode(cpb1, OUTPUT);
    pinMode(go1, INPUT);
    pinMode(go2, INPUT);
    pinMode(go3, INPUT);

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

    // Motor Driver Pins //
    pinMode(enableL, OUTPUT);
    pinMode(enableR, OUTPUT);
    pinMode(dirLF, OUTPUT);
    pinMode(dirLR, OUTPUT);
    pinMode(dirRF, OUTPUT);
    pinMode(dirRR, OUTPUT);
}

// end of setup //

// start of loop //

void loop()
{
    lineSensor();
    //serialRead();
    infraredSensor();
    rfid();
    rfidTest();
    motorDev();
    comm();
    commTest();
    //motors();
}
// end of loop //

void motors()
{
    switch (go)
    {
   
    case 0:
    stop();
    break;

    case 1:
    lineFollow();
    break;

    case 2:
    forward();
    break;
    
    case 3:
    reverse();
    break;

    case 4:
    left();
    break;

    case 5:
    right();
    break;

    
    default:
        break;
    }

}
void commTest()
{
    Serial.print("speed: ");
    Serial.print(speed);
    Serial.print("    Go: ");
    Serial.println(go);
}

void comm()
{
    speed = (digitalRead(speedb0) + 2 * digitalRead(speedb1)) * 85;

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

    go = digitalRead(go1) + digitalRead(go2)*2 + digitalRead(go3)*4;
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

void rfidTest()
{
    Serial.print("Checkpoint:");
    Serial.print(checkpoint);
    Serial.println(" ");
}

void infraredSensor()
{
    objectDetected = !digitalRead(irPin);
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

// Movement Commands //

void lineFollow()
{

}
void forward()
{
    analogWrite(enableL, speed);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void reverse()
{
    analogWrite(enableL, speed);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, LOW);
    digitalWrite(dirLR, HIGH);
    digitalWrite(dirRR, HIGH);
}

void left()
{
    analogWrite(enableL, 0);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void right()
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 0);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void stop()
{
    analogWrite(enableL, 0);
    analogWrite(enableR, 0);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, LOW);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
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
    reverse();
    delay(2000);
    /*
    reverse();
    delay(2000);
    left();
    delay(2000);
    right();
    delay(2000);
    */
    stop();
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