#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

// I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <U8g2lib.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

#include <SPI.h>
#include <LoRa.h>
#include "FS.h" 
#include "SD.h"

// --- WiFi ---
const char* ssid = "xxxxxx";
const char* password = "xxxxxx";

// Pins
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 41
#define IMU_ADDRESS  0x29
#define LCD_ADDRESS  0x3C
#define LORA_SCK  13
#define LORA_MISO 12
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_RST  14
#define LORA_IRQ  3
#define SD_CS     38
#define PRESSURE_PIN 2
#define IN1 17
#define IN2 18
#define IN3 43
#define IN4 44
#define IN5 1
#define IN6 9
#define IN7 47
#define IN8 21
#define IN9 4 
#define IN10 5 
#define IN11 6 
#define IN12 7 
#define BUTTON_PIN_1 39
#define BUTTON_PIN_2 40
#define GPS_RX_PIN 15

// lora config
#define BOARD_ID "SUBMARINE" // Receiver
#define LORA_FREQUENCY 433.2E6
#define SPREADING_FACTOR 11
#define CODING_RATE 5
#define SYNC_WORD 0x36


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, IMU_ADDRESS);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

volatile bool button1_pressed = false;
volatile bool button2_pressed = false;
float currentVelocityX=0.0, currentVelocityY=0.0, currentVelocityZ=0.0;
float currentDisplacementX=0.0, currentDisplacementY=0.0, currentDisplacementZ=0.0;
unsigned long previousMillis = 0;
const float ACCEL_NOISE_THRESHOLD = 0.05;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000;
int packetCounter = 0;
const float OffSet = 0.47;
float V1, P1, raw_val1;       
float V2, P2, raw_val2;
float rho, g, H1, H2;
const float Vmin = 0.5;
const float Vmax = 4.5;
const float fullRangePSI = 100.0;
const float psiToKpa = 6.89476; 


void IRAM_ATTR isr_button_1() { button1_pressed = true; }
void IRAM_ATTR isr_button_2() { button2_pressed = true; }
void selectLoRa()   { digitalWrite(SD_CS, HIGH); digitalWrite(LORA_CS, LOW); }
void selectSDCard() { digitalWrite(LORA_CS, HIGH); digitalWrite(SD_CS, LOW); }
void freeCS()       { digitalWrite(LORA_CS, HIGH); digitalWrite(SD_CS, HIGH); }
void appendFile(fs::FS &fs, const char * path, const char * message){ File file = fs.open(path, FILE_APPEND); if(!file) return; file.print(message); file.close(); }

void controlMotor(int motor, String action) {
  int inA, inB;
  if (motor == 1) { inA = IN1; inB = IN2; } 
  else if (motor == 2) { inA = IN3; inB = IN4; } 
  else if (motor == 3) { inA = IN5; inB = IN6; } 
  else if (motor == 4) { inA = IN7; inB = IN8; } 
  else if (motor == 5) { inA = IN9; inB = IN10; } 
  else if (motor == 6) { inA = IN11; inB = IN12; } 
  else { Serial.println("❌ Invalid motor number received."); return; }

  if (action == "forward") { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); Serial.printf("Motor %d -> forward\n", motor); } 
  else if (action == "reverse") { digitalWrite(inA, LOW); digitalWrite(inB, HIGH); Serial.printf("Motor %d <- reverse\n", motor); } 
  else if (action == "stop") { digitalWrite(inA, LOW); digitalWrite(inB, LOW); Serial.printf("Motor %d [] stopped\n", motor); } 
  else { Serial.println("❌ Invalid motor action received."); }
}

void updateSensorData(sensors_vec_t acceleration, float deltaTime) { float accelMagnitude = sqrt(pow(acceleration.x, 2) + pow(acceleration.y, 2) + pow(acceleration.z, 2)); if (accelMagnitude < ACCEL_NOISE_THRESHOLD) { currentVelocityX = 0.0; currentVelocityY = 0.0; currentVelocityZ = 0.0; } else { currentVelocityX += acceleration.x * deltaTime; currentVelocityY += acceleration.y * deltaTime; currentVelocityZ += acceleration.z * deltaTime; } currentDisplacementX += currentVelocityX * deltaTime; currentDisplacementY += currentVelocityY * deltaTime; currentDisplacementZ += currentDisplacementZ * deltaTime; }

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n---Booting---");
  rho = 998;
  g = 9.81;
  analogReadResolution(12);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(IN9, OUTPUT); pinMode(IN10, OUTPUT); pinMode(IN11, OUTPUT); pinMode(IN12, OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT_PULLUP); pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_1), isr_button_1, FALLING); attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_2), isr_button_2, FALLING);
  Serial.println("Interrupts attached.");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1); // RX only
  Serial.println("GPS Serial Port Initialized.");

  
  // I2C and SPI setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  u8g2.setI2CAddress(LCD_ADDRESS * 2); u8g2.begin();
  if (!bno.begin()) { Serial.println("BNO055 FAILED!"); while (1); }
  bno.setExtCrystalUse(true);
  pinMode(LORA_CS, OUTPUT); pinMode(SD_CS, OUTPUT); freeCS();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  selectLoRa();
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); LoRa.setSpreadingFactor(SPREADING_FACTOR); LoRa.setSyncWord(SYNC_WORD); LoRa.setCodingRate4(CODING_RATE);
  if (!LoRa.begin(LORA_FREQUENCY)) { Serial.println("LoRa FAILED!"); while (1); }
  freeCS();
  selectSDCard();
  if (!SD.begin(SD_CS)) { Serial.println("SD Card FAILED!"); } else { appendFile(SD, "/system_log.txt", "------ System Boot OK ------\r\n"); }
  freeCS();
  Serial.println("All peripherals initialized.");

  // Calibration
  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x12_tr); u8g2.drawStr(0, 15, "Calibrating IMU..."); u8g2.sendBuffer();
  uint8_t system_cal;
  do { bno.getCalibration(&system_cal, nullptr, nullptr, nullptr); delay(100); } while (system_cal < 3);
  Serial.println("IMU Calibrated!");

  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(0, 32, "System Ready"); u8g2.sendBuffer();
  delay(1000);
  previousMillis = millis();
}

void loop() {
  ArduinoOTA.handle();

  // Limit switch
  if (button1_pressed) { Serial.println("! BTN 1 - STOP M3 !"); controlMotor(3, "stop"); button1_pressed = false; }
  if (button2_pressed) { Serial.println("! BTN 2 - STOP M4 !"); controlMotor(4, "stop"); button2_pressed = false; }

  selectLoRa();
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedCommand = "";
    while (LoRa.available()) {
      receivedCommand += (char)LoRa.read();
    }
    
    Serial.print("\n<<< LoRa Command Received: '");
    Serial.print(receivedCommand);
    Serial.println("'");
    
    // Parse commands
    receivedCommand.trim();
    receivedCommand.toLowerCase();
    int spaceIndex = receivedCommand.indexOf(' ');
    if (spaceIndex != -1) {
      String motorStr = receivedCommand.substring(0, spaceIndex);
      String action = receivedCommand.substring(spaceIndex + 1);
      controlMotor(motorStr.toInt(), action);
    } else {
      Serial.println("⚠️ Invalid LoRa command format received.");
    }
  }
  freeCS();
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  unsigned long currentMillis = millis();
  float delta_t = (currentMillis - previousMillis) / 1000.0f;
  previousMillis = currentMillis;
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  if (delta_t > 0) { updateSensorData(linearAccelData.acceleration, delta_t); }

  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;
    packetCounter++;

    sensors_event_t eulerData, gyroData; imu::Quaternion quat;
    bno.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    quat = bno.getQuat();
    u8g2.clearBuffer();
    char buffer[64];
    u8g2.setFont(u8g2_font_6x12_tr);
    sprintf(buffer, "H:%.1f R:%.1f P:%.1f", eulerData.orientation.x, eulerData.orientation.y, eulerData.orientation.z);
    u8g2.drawStr(0, 12, buffer);
    sprintf(buffer, "ACC Z: %.2f", linearAccelData.acceleration.z);
    u8g2.drawStr(0, 28, buffer);
    sprintf(buffer, "VEL Z: %.2f", currentVelocityZ);/Users/miniexploit/Documents/GPSTracker/server.py
    u8g2.drawStr(0, 44, buffer);
    sprintf(buffer, "DIS Z: %.2f", currentDisplacementZ);
    u8g2.drawStr(0, 60, buffer);
    u8g2.sendBuffer();
    raw_val1 = analogRead(PRESSURE_PIN);
    V1 = raw_val1 * (3.3 / 4095);
    float pressureKPa = 0.0;

    if (V1 <= Vmin) {
      pressureKPa = 0.0;
    } else if (V1 >= Vmax) {
      pressureKPa = fullRangePSI * psiToKpa;
    } else {
      float psi = ((V1 - Vmin) / (Vmax - Vmin)) * fullRangePSI;
      pressureKPa = psi * psiToKpa;
    }

    P1 = pressureKPa;                // in kPa
    H1 = (P1 * 1000.0) / (rho * g);

    char loraPacket[250];
    snprintf(loraPacket, sizeof(loraPacket),
             "LAT:%.6f,LON:%.6f|P:%.2fkPa|Depth:%.2fm|ACC:%.2f,%.2f,%.2f|EUL:%.2f,%.2f,%.2f|GYR:%.2f,%.2f,%.2f|Q:%.2f,%.2f,%.2f,%.2f|VEL:%.2f,%.2f,%.2f|DIS:%.2f,%.2f,%.2f",
             gps.location.lat(), gps.location.lng(), // Added GPS coordinates
             P1, H1,
             linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z,
             eulerData.orientation.x, eulerData.orientation.y, eulerData.orientation.z,
             gyroData.gyro.x, gyroData.gyro.y, gyroData.gyro.z,
             quat.w(), quat.x(), quat.y(), quat.z(),
             currentVelocityX, currentVelocityY, currentVelocityZ,
             currentDisplacementX, currentDisplacementY, currentDisplacementZ);
    selectLoRa();
    LoRa.beginPacket(); LoRa.print(loraPacket); LoRa.endPacket();
    freeCS();
    // Serial.println("\n>>> Sent LoRa Data Packet #" + String(packetCounter));
    //  Serial.println(loraPacket);
    Serial.print("kPa:");
    Serial.print(P1);
    Serial.print(" ");
    Serial.print("Depth(m):");
    Serial.println(H1);
  
  }
}