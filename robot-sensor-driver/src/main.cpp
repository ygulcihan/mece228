#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <MFRC522.h>
#include <RH_ASK.h>

// Rf Communication //
RH_ASK rf(2000, 30, 31, 32, true);
bool c1sent = false;
bool c2sent = false;
bool c3sent = false;

// Line Sensor Variables //
int read0, read1, read2, read3, read4, read5, read6, read7;
String color0, color1, color2, color3, color4, color5, color6, color7;
#define lc "white"

// Communication Pins & Variables //
#define speedb1 25
#define speedb0 27
#define cpb1 2
#define cpb0 3
#define go1 22
#define go2 23
#define go3 24
unsigned int mspeed = 175;
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
void rfSend(String rfMessage);
void motors();

void lineFollow();
void forward();
void reverse();
void left();
void right();
void stop();

void goLeft1(unsigned int speed);
void goLeft2(unsigned int speed);
void goLeft3(unsigned int speed);
void goRight1(unsigned int speed);
void goRight2(unsigned int speed);
void goRight3(unsigned int speed);
void goStraight(unsigned int speed);

// start of setup //
void setup()
{
    Serial.begin(9600);
    SPI.begin();
    mfrc522.PCD_Init();

    if(!rf.init())
    {
        Serial.println("rf init failed");
    }

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
    lineTest();
    //serialRead();
    infraredSensor();
    rfid();
    //rfidTest();
    //motorDev();
    comm();
    //commTest();
    motors();
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
    comm();
    Serial.print("mspeed: ");
    Serial.print(mspeed);
    Serial.print("    Go: ");
    Serial.println(go);
}

void comm()
{
    //speed = ((digitalRead(speedb0) + 2 * digitalRead(speedb1)) * 60) + 75;

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

    go = digitalRead(go1) + digitalRead(go2) * 2 + digitalRead(go3) * 4;
}

void rfSend(const char *rfMessage)
{
    rf.send((uint8_t*)rfMessage, strlen(rfMessage));
    rf.waitPacketSent();
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
        if (dly1 - dl1p >= 6000)
        {
            c1sent = false;
            dl1p = dly1;
        }

        if(!c1sent)
        {
            rfSend("c1");
            c1sent = true;
        }
    }

    else if (content.substring(1) == "79 CB 2F 5A")
    {
        checkpoint = 2;

        unsigned long dly2 = millis();
        if (dly2 - dl2p >= 6000)
        {
            c2sent = false;
            dl2p = dly2;
        }
        
        if(!c2sent)
        {
            rfSend("c2");
            c2sent = true;
        }
    }

    else if (content.substring(1) == "59 9D A4 5A")
    {
        checkpoint = 3;

        unsigned long dly3 = millis();
        if (dly3 - dl3p >= 3000)
        {
            c3sent = false;
            dl3p = dly3;
        }

        if(!c3sent)
        {
            rfSend("c3");
            c3sent = true;
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
    if (color0 == lc)
    {
        goLeft3(170);
    }
    else if (color7 == lc)
    {
        goRight3(170);
    }

    else if (color1 == lc)
    {
        goLeft2(170);
    }
    else if (color6 == lc)
    {
        goRight2(170);
    }
    else if (color2 == lc)
    {
        goLeft1(170);
    }
    else if (color5 == lc)
    {
        goRight1(170);
    }
    else if (color3 == lc || color4 == lc)
    {
        goStraight(170);
    }
}
void forward()
{
    analogWrite(enableL, mspeed);
    analogWrite(enableR, mspeed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void reverse()
{
    analogWrite(enableL, mspeed);
    analogWrite(enableR, mspeed);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, LOW);
    digitalWrite(dirLR, HIGH);
    digitalWrite(dirRR, HIGH);
}

void left()
{
    digitalWrite(enableL, LOW);
    analogWrite(enableR, mspeed);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void right()
{
    analogWrite(enableL, mspeed);
    digitalWrite(enableR, LOW);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, LOW);
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

void goStraight(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goLeft1(unsigned int speed)
{
    analogWrite(enableL, 130);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight1(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 130);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goLeft2(unsigned int speed)
{
    analogWrite(enableL, 80);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight2(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 80);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goLeft3(unsigned int speed)
{
    analogWrite(enableL, 50);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight3(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 50);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
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
    read0 = analogRead(A7);
    if (read0 <= 900)
    {
        color0 = "white";
    }
    else
    {
        color0 = "black";
    }

    read1 = analogRead(A6);
    if (read1 <= 900)
    {
        color1 = "white";
    }
    else
    {
        color1 = "black";
    }

    read2 = analogRead(A5);
    if (read2 <= 900)
    {
        color2 = "white";
    }
    else
    {
        color2 = "black";
    }

    read3 = analogRead(A4);
    if (read3 <= 900)
    {
        color3 = "white";
    }
    else
    {
        color3 = "black";
    }

    read4 = analogRead(A3);
    if (read4 <= 900)
    {
        color4 = "white";
    }
    else
    {
        color4 = "black";
    }

    read5 = analogRead(A2);
    if (read5 <= 900)
    {
        color5 = "white";
    }
    else
    {
        color5 = "black";
    }

    read6 = analogRead(A1);
    if (read6 <= 900)
    {
        color6 = "white";
    }
    else
    {
        color6 = "black";
    }

    read7 = analogRead(A0);
    if (read7 <= 900)
    {
        color7 = "white";
    }
    else
    {
        color7 = "black";
    }
}

void lineTest()
{
    Serial.print("0: ");
    Serial.print(read0);
    Serial.print(",");
    Serial.print(color0);
    Serial.print("  ");

    Serial.print("1: ");
    Serial.print(read1);
    Serial.print(",");
    Serial.print(color1);
    Serial.print("  ");

    Serial.print("2: ");
    Serial.print(read2);
    Serial.print(",");
    Serial.print(color2);
    Serial.print("  ");

    Serial.print("3: ");
    Serial.print(read3);
    Serial.print(",");
    Serial.print(color3);
    Serial.print("  ");

    Serial.print("4: ");
    Serial.print(read4);
    Serial.print(",");
    Serial.print(color4);
    Serial.print("  ");

    Serial.print("5: ");
    Serial.print(read5);
    Serial.print(",");
    Serial.print(color5);
    Serial.print("  ");

    Serial.print("6: ");
    Serial.print(read6);
    Serial.print(",");
    Serial.print(color6);
    Serial.print("  ");

    Serial.print("7: ");
    Serial.print(read7);
    Serial.print(",");
    Serial.print(color7);
    Serial.println("  ");
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