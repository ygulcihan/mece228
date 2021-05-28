#include <Arduino.h>
#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"
#define SSID1 "ASUS2"
#define PWD1 "mTx.96,tGb38"
#define SSID2 "cilgin robot 3.0"
#define PWD2 "12345678"
#define redLed 33
#define camFlash 4

#define speedb1 0
#define speedb0 16
#define cpb1 14
#define cpb0 15
#define go1 2
#define go2 12
#define go3 13

unsigned int speed = 1;
unsigned int checkpoint = 0;

uint32_t Freq = 0;

OV2640 cam;

WebServer server(80);

IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

const char HEADER[] = "HTTP/1.1 200 OK\r\n"
                      "Access-Control-Allow-Origin: *\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void handle_jpg_stream();
void handle_jpg();
void handleNotFound();
void ledOn();
void ledOff();
void speed1();
void speed2();
void speed3();
void checkClocks();
void comm();
void include();
void forward();
void stop();
void lineFollow();
void reverse();
void left();
void right();

void setup()
{
  btStop();

  Serial.begin(115200);

  if (!WiFi.config(local_IP, gateway, subnet))
  {
    Serial.println("STA Failed to configure");
  }

  checkClocks();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  cam.init(config);

  IPAddress ip;

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(SSID2, PWD2);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }

  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  Serial.println("");
  Serial.print("IP address for network ");
  Serial.print(SSID2);
  Serial.print(" : ");
  Serial.print(WiFi.softAPIP());

  server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.on("/ledOn", HTTP_GET, ledOn);
  server.on("/ledOff", HTTP_GET, ledOff);
  server.on("/speed1", HTTP_GET, speed1);
  server.on("/speed2", HTTP_GET, speed2);
  server.on("/speed3", HTTP_GET, speed3);
  server.on("/forward", HTTP_GET, forward);
  server.on("/lineFollow", HTTP_GET, lineFollow);
  server.on("/stop", HTTP_GET, stop);
  server.on("/reverse", HTTP_GET, reverse);
  server.on("/left", HTTP_GET, left);
  server.on("/right", HTTP_GET, right);
  server.onNotFound(handleNotFound);
  server.begin();

  pinMode(redLed, OUTPUT);
  pinMode(camFlash, OUTPUT);
  digitalWrite(camFlash, LOW);
  digitalWrite(redLed, HIGH);

  pinMode(cpb1, INPUT);
  pinMode(cpb0, INPUT);
  pinMode(speedb1, OUTPUT);
  pinMode(speedb0, OUTPUT);
  pinMode(go1, OUTPUT);
  pinMode(go2, OUTPUT);
  pinMode(go3, OUTPUT);
}

void loop()
{
  include();
}

void handle_jpg_stream(void)
{
  char buf[32];
  int s;

  WiFiClient client = server.client();

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  digitalWrite(redLed, LOW);

  while (true)
  {
    include();
    if (!client.connected())
    {
      digitalWrite(redLed, HIGH);
      break;
    }
    cam.run();
    s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    sprintf(buf, "%d\r\n\r\n", s);
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n"
                       "Content-disposition: inline; filename=capture.jpg\r\n"
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void)
{
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected())
    return;

  client.write(JHEADER, jhdLen);
  client.write((char *)cam.getfb(), cam.getSize());

  digitalWrite(redLed, LOW);
  delay(50);
  digitalWrite(redLed, HIGH);
  delay(50);
  digitalWrite(redLed, LOW);
  delay(50);
  digitalWrite(redLed, HIGH);
}

void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

void ledOn()
{
  digitalWrite(camFlash, HIGH);
  server.send(200, "text / plain", "ledOn");
}

void ledOff()
{
  digitalWrite(camFlash, LOW);
  server.send(200, "text / plain", "ledOff");
}

void speed1()
{
  speed = 1;
  server.send(200, "text / plain", "speed1");
}

void speed2()
{
  speed = 2;
  server.send(200, "text / plain", "speed2");
}

void speed3()
{
  speed = 3;
  server.send(200, "text / plain", "speed3");
}

void checkClocks()
{
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
}

void comm()
{

  checkpoint = digitalRead(cpb0) + digitalRead(cpb1) * 2;

  bool spd1, spd0;

  switch (speed)
  {
  case 1:
    spd1 = 0;
    spd0 = 1;
    break;

  case 2:
    spd1 = 1;
    spd0 = 0;
    break;

  case 3:
    spd1 = 1;
    spd0 = 1;
    break;

  default:
    spd1 = 0;
    spd0 = 0;
    break;
  }

  digitalWrite(speedb1, spd1);
  digitalWrite(speedb0, spd0);
}

void include()
{
  server.handleClient();
  comm();
}

void stop()
{
  digitalWrite(go1, LOW);
  digitalWrite(go2, LOW);
  digitalWrite(go3, LOW);
}

void lineFollow()
{
  digitalWrite(go1, HIGH);
  digitalWrite(go2, LOW);
  digitalWrite(go3, LOW);
}

void forward()
{
  digitalWrite(go1, LOW);
  digitalWrite(go2, HIGH);
  digitalWrite(go3, LOW);
}

void reverse()
{
  digitalWrite(go1, HIGH);
  digitalWrite(go2, HIGH);
  digitalWrite(go3, LOW);
}

void left()
{
  digitalWrite(go1, LOW);
  digitalWrite(go2, LOW);
  digitalWrite(go3, HIGH);
}

void right()
{
  digitalWrite(go1, HIGH);
  digitalWrite(go2, LOW);
  digitalWrite(go3, HIGH);
}