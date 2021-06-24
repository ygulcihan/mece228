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
bool stopSent = false;
unsigned int stopSendCount = 0;

// Line Sensor Variables //
bool L4, L3, L2, L1, ML, MR, R1, R2, R3, R4, lc;
#define lineColor "white"
#define L4P A15
#define L3P A14
#define L2P A13
#define L1P A12
#define MLP A11
#define MRP A4
#define R1P A3
#define R2P A2
#define R3P A1
#define R4P A0

if (lineColor == "white" || lineColor == "White" || lineColor == "WHITE")
{
    lc = 1;
}

else
{
    lc = 0;
}

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

// Ultrasonic Sensor Variables //
#define trigPin 47
#define echoPin 46
unsigned long duration;
unsigned int distance;
unsigned int totalDistance = 0;
unsigned int a = 0;

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
void ultrasonicTest();
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

void goLeftML(unsigned int speed);
void goLeft1(unsigned int speed);
void goLeft2(unsigned int speed);
void goLeft3(unsigned int speed);
void goLeft4(unsigned int speed);
void goRightMR(unsigned int speed);
void goRight1(unsigned int speed);
void goRight2(unsigned int speed);
void goRight3(unsigned int speed);
void goRight4(unsigned int speed);
void goStraight(unsigned int speed);

// start of setup //
void setup()
{
    Serial.begin(9600);
    SPI.begin();
    mfrc522.PCD_Init();

    if (!rf.init())
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
    pinMode(R4, INPUT);
    pinMode(R3, INPUT);
    pinMode(R2, INPUT);
    pinMode(R1, INPUT);
    pinMode(MR, INPUT);
    pinMode(ML, INPUT);
    pinMode(L1, INPUT);
    pinMode(L2, INPUT);
    pinMode(L3, INPUT);
    pinMode(L4, INPUT);

    // Ultrasonic Sensor Pins //
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

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
    //lineTest();
    //serialRead();
    rfid();
    //rfidTest();
    //motorDev();
    comm();
    //commTest();
    motors();
    ultrasonicSensor();
    //ultrasonicTest();
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
    rf.send((uint8_t *)rfMessage, strlen(rfMessage));
    rf.waitPacketSent();
}

void ultrasonicSensor()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    if (distance <= 15 && !stopSent)
    {
        stop();
        rfSend("stp");
        stopSendCount++;

        if (stopSendCount >= 3)
        {
            stopSent = true;
        }
    }

    else if (stopSent && distance > 15)
    {
        rfSend("clr");
        stopSent = false;
        stopSendCount = 0;
    }
}

void ultrasonicTest()
{
    ultrasonicSensor();
    Serial.print("Distance: ");
    Serial.println(distance);
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

        if (!c1sent)
        {
            rfSend("c1x");
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

        if (!c2sent)
        {
            rfSend("c2x");
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

        if (!c3sent)
        {
            rfSend("c3x");
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

// Movement Commands //

void lineFollow()
{
    if(L4 == lc)
    {
        goLeft4(100);
    }

    if(R4 == lc)
    {
        goRight4(100);
    }

    if(L3 == lc)
    {
        goLeft3(100);
    }

    if(R3 == lc)
    {
        goRight3(100);
    }

    if(L2 == lc)
    {
        goLeft2(100);
    }

    if(R2 == lc)
    {
        goRight2(100);
    }

    if(L1 == lc)
    {
        goLeft1(100);
    }

    if(R1 == lc)
    {
        goRight1(100);
    }

    if(ML == lc)
    {
        goLeftML(100);
    }

    if(MR == lc)
    {
        goRightMR(100);
    }
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
    analogWrite(enableL, 50);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight1(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 50);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goLeft2(unsigned int speed)
{
    analogWrite(enableL, 20);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight2(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, 20);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goLeft3(unsigned int speed)
{
    analogWrite(enableL, speed - 30);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
}

void goRight3(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, speed - 30);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, LOW);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, HIGH);
}

void goRight4(unsigned int speed)
{
    analogWrite(enableL, speed);
    analogWrite(enableR, speed - 30);

    digitalWrite(dirLF, HIGH);
    digitalWrite(dirRF, LOW);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, HIGH);
}

void goLeft4(unsigned int speed)
{
    analogWrite(enableL, speed - 30);
    analogWrite(enableR, speed);

    digitalWrite(dirLF, LOW);
    digitalWrite(dirRF, HIGH);
    digitalWrite(dirLR, LOW);
    digitalWrite(dirRR, LOW);
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
}

void lineSensor()
{
    R4 = digitalRead(R4P);
    R3 = digitalRead(R3P);
    R2 = digitalRead(R2P);
    R1 = digitalRead(R1P);
    MR = digitalRead(MRP);
    ML = digitalRead(MLP);
    L1 = digitalRead(L1P);
    L2 = digitalRead(L2P);
    L3 = digitalRead(L3P);
    L4 = digitalRead(L4P);
}

void lineTest()
{
    Serial.print("L4:");
    Serial.print(L4);
    Serial.print(" ");

    Serial.print("L3:");
    Serial.print(L3);
    Serial.print(" ");

    Serial.print("L2:");
    Serial.print(L2);
    Serial.print(" ");

    Serial.print("L1:");
    Serial.print(L1);
    Serial.print(" ");

    Serial.print("ML:");
    Serial.print(ML);
    Serial.print(" ");

    Serial.print("MR:");
    Serial.print(MR);
    Serial.print(" ");

    Serial.print("R1:");
    Serial.print(R1);
    Serial.print(" ");

    Serial.print("R2:");
    Serial.print(R3);
    Serial.print(" ");

    Serial.print("R3:");
    Serial.print(R3);
    Serial.print(" ");

    Serial.print("R4:");
    Serial.print(R4);
    Serial.println(" ");
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