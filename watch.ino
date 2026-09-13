#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <U8g2lib.h>
#include <WiFi.h>
#include <ESP_Mail_Client.h>
#include "DHT.h"
#include <MPU6050_light.h>

// ---------------- Pin Configuration ----------------
#define BUZZER_PIN 23
#define BUTTON_PIN 15
#define VIBRATION_PIN 18
#define DHTPIN 4
#define DHTTYPE DHT11

// Realistic Thresholds for Emergency Alerts
#define FEVER_THRESHOLD 37.5  // °C
#define MIN_HR 50             // bpm
#define MAX_HR 120            // bpm
#define MIN_SPO2 92           // %

// ---------------- WiFi & Email Config ----------------
const char* ssid = "iQOO Z9s 5G";
const char* password = "davisthomas10";
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 587
#define AUTHOR_EMAIL "davisthomasmc@gmail.com"
#define AUTHOR_PASSWORD "ltrn xknv elgg lnwb"
// Sent directly to the patient himself!
#define RECIPIENT_EMAIL "davisthomasmc@gmail.com"

// ---------------- Objects ----------------
MAX30105 particleSensor;
DHT dht(DHTPIN, DHTTYPE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MPU6050 mpu(Wire);
SMTPSession smtp;

// ---------------- System States & Timers ----------------
enum AlertState { IDLE, OBSERVING, VIBRATING, BUZZING, ALERT_SENT };
AlertState currentState = IDLE;

unsigned long stateTimer = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastDHTUpdate = 0;
unsigned long lastBeatTime = 0;

float beatsPerMinute = 0;
int beatAvg = 0;
int spo2 = 0;
float temperature = 0.0;
bool fingerDetected = false;

// Heart Rate Moving Average Buffer
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;

// Fall Detection Variables
bool fallDetected = false;
bool jerkDetected = false;
unsigned long jerkTime = 0;

// ---------------- Function Declarations ----------------
void sendEmailAlert(String reason);
void handleAlertSequence();
void resetSystem();
void checkSensors();
void checkFallDetection();
void updateDisplay();

void smtpCallback(SMTP_Status status) { 
  Serial.println(status.info()); 
}

void setup() {
  Serial.begin(115200);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIBRATION_PIN, OUTPUT);
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIBRATION_PIN, LOW);

  // Initialize I2C at standard 100kHz for stability across all devices
  Wire.begin();
  Wire.setClock(100000);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0, 20, "Waking up watch...");
  u8g2.drawStr(0, 38, "Connecting WiFi...");
  u8g2.sendBuffer();

  WiFi.begin(ssid, password);
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
  }

  dht.begin();

  // Pulse Oximeter Initialization
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30105 initialization failed!");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 20, "MAX30105 DEAD!");
    u8g2.drawStr(0, 40, "Check Wiring...");
    u8g2.sendBuffer();
    while (1);
  }

  // Setup MAX30105 Configuration
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); // Low power to prevent saturation
  particleSensor.setPulseAmplitudeGreen(0);

  // MPU6050 Setup
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 status failed: ");
    Serial.println(status);
    u8g2.clearBuffer();
    u8g2.drawStr(0, 20, "MPU6050 DEAD!");
    u8g2.sendBuffer();
    while (1);
  }
  delay(500);
  mpu.calcOffsets(); // Ensure sensor is placed flat during startup!

  smtp.callback(smtpCallback);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, "System Ready!");
  u8g2.drawStr(0, 38, "Ready to judge...");
  u8g2.sendBuffer();
  delay(1000);
}

void loop() {
  // 1. Always update motion hardware
  mpu.update();

  // 2. Read Sensors (Non-blocking)
  checkSensors();
  checkFallDetection();

  // 3. Emergency Cancel Button Override
  if (digitalRead(BUTTON_PIN) == LOW) {
    resetSystem();
    delay(300); // Debounce button
  }

  // 4. State Machine Evaluation
  handleAlertSequence();

  // 5. Non-blocking Screen Updates (200ms refresh rate)
  if (millis() - lastDisplayUpdate > 200) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
}

// ---------------- Sensor Data Processing ----------------
void checkSensors() {
  // Read DHT11 non-blocking every 2 seconds
  if (millis() - lastDHTUpdate > 2000) {
    lastDHTUpdate = millis();
    float t = dht.readTemperature();
    if (!isnan(t)) {
      temperature = t;
    }
  }

  // Read MAX30105 IR intensity
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    fingerDetected = false;
    beatAvg = 0;
    spo2 = 0;
    return;
  }

  fingerDetected = true;

  // Heartbeat Detection
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeatTime;
    lastBeatTime = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 30 && beatsPerMinute < 220) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // Algorithmic estimation for hackathon display demonstration
  if (beatAvg > 0) {
    spo2 = map(constrain(irValue, 50000, 120000), 50000, 120000, 95, 99);
  }
}

// ---------------- Motion / Fall Detection ----------------
void checkFallDetection() {
  float acc = sqrt(mpu.getAccX() * mpu.getAccX() +
                   mpu.getAccY() * mpu.getAccY() +
                   mpu.getAccZ() * mpu.getAccZ());

  if (acc > 2.8 && !jerkDetected) { // High threshold for high impact impact
    jerkDetected = true;
    jerkTime = millis();
  }

  if (jerkDetected && (millis() - jerkTime > 1000)) {
    if (acc < 1.1) { // Stationary after spike confirms fall
      fallDetected = true;
      jerkDetected = false;
    } else {
      jerkDetected = false;
    }
  }
}

// ---------------- Alert Sequence State Handler ----------------
void handleAlertSequence() {
  bool vitalsAbnormal = false;

  if (fingerDetected && beatAvg > 0) {
    if (beatAvg < MIN_HR || beatAvg > MAX_HR || spo2 < MIN_SPO2 || temperature >= FEVER_THRESHOLD) {
      vitalsAbnormal = true;
    }
  }

  switch (currentState) {
    case IDLE:
      if (vitalsAbnormal || fallDetected) {
        currentState = OBSERVING;
        stateTimer = millis();
      }
      break;

    case OBSERVING:
      if (fallDetected || (vitalsAbnormal && (millis() - stateTimer >= 10000))) {
        currentState = VIBRATING;
        stateTimer = millis();
        digitalWrite(VIBRATION_PIN, HIGH);
      } else if (!vitalsAbnormal && !fallDetected) {
        currentState = IDLE;
      }
      break;

    case VIBRATING:
      // Vibration active for 10 SECONDS
      if (millis() - stateTimer >= 10000) {
        digitalWrite(VIBRATION_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH);
        currentState = BUZZING;
        stateTimer = millis();
      }
      break;

    case BUZZING:
      // Buzzer active for 10 SECONDS
      if (millis() - stateTimer >= 10000) {
        digitalWrite(BUZZER_PIN, LOW);
        String reason = fallDetected ? "Gravity Wins Again! (Hard Fall)" : "System Overheat / Abnormal Vitals";
        sendEmailAlert(reason);
        currentState = ALERT_SENT;
        stateTimer = millis();
      }
      break;

    case ALERT_SENT:
      if (millis() - stateTimer >= 15000) {
        resetSystem();
      }
      break;
  }
}

void resetSystem() {
  currentState = IDLE;
  fallDetected = false;
  jerkDetected = false;
  digitalWrite(VIBRATION_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

// ---------------- OLED Display Renderer ----------------
void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);

  if (!fingerDetected) {
    u8g2.drawStr(0, 16, "I feel lonely...");
    u8g2.drawStr(0, 32, "I need human contact!");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 52, "Wear your watch...");
  } else {
    u8g2.setCursor(0, 12); 
    u8g2.print("HR  : "); u8g2.print(beatAvg); u8g2.print(" bpm");

    u8g2.setCursor(0, 26); 
    u8g2.print("SpO2: "); u8g2.print(spo2); u8g2.print(" %");

    u8g2.setCursor(0, 40); 
    if (temperature >= FEVER_THRESHOLD) {
      u8g2.print("YOU ARE SO HOT!");
    } else {
      u8g2.print("Temp: "); u8g2.print(temperature, 1); u8g2.print(" C");
    }

    // Banner status display logic
    u8g2.setFont(u8g2_font_5x7_tf);
    if (fallDetected) {
      u8g2.drawStr(0, 58, "GRAVITY WINS AGAIN!");
    } else {
      switch (currentState) {
        case IDLE:
          u8g2.drawStr(0, 58, "Status: Monitoring...");
          break;
        case OBSERVING:
          u8g2.drawStr(0, 58, "STILL ALIVE...");
          break;
        case VIBRATING:
          u8g2.drawStr(0, 58, "WARNING: VIBRATING!");
          break;
        case BUZZING:
          u8g2.drawStr(0, 58, "ALERT: BUZZER ACTIVE!");
          break;
        case ALERT_SENT:
          u8g2.drawStr(0, 58, "NOW EVERYONE KNOWS...");
          break;
      }
    }
  }

  u8g2.sendBuffer();
}

// ---------------- Riddling Emergency Email Dispatcher ----------------
void sendEmailAlert(String reason) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Skipping Email.");
    return;
  }

  SMTP_Message email;
  email.sender.name = "ESP32 Useless Watch";
  email.sender.email = AUTHOR_EMAIL;
  email.subject = "RIDDLE ME THIS: Self-Diagnostic Alert!";
  email.addRecipient("Patient (You)", RECIPIENT_EMAIL);

  // Funny riddling self-email content
  String body = "--- ANOMALY DETECTED BY YOUR USELESS WATCH ---\n\n";
  body += "Dear Patient (You),\n\n";
  body += "Riddle me this...\n";
  body += "I sit on your wrist, I screamed at you for 20 seconds straight (10s vibration + 10s buzzer),\n";
  body += "and now I am sending an emergency email to the exact same person who triggered it...\n";
  body += "Who am I?\n\n";
  body += "Answer: Your Useless Health Watch making sure you know you triggered your own alert!\n\n";
  body += "[ALERT DETAILS]\n";
  body += "Primary Incident: " + reason + "\n\n";
  body += "[YOUR LIVE VITALS]\n";
  body += "• Heart Rate: " + String(beatAvg) + " BPM (Your heart is still ticking!)\n";
  body += "• Oxygen (SpO2): " + String(spo2) + " %\n";
  body += "• Temperature: " + String(temperature, 1) + " C\n\n";
  body += "Now everyone knows... starting with YOU!\n";
  
  email.text.content = body.c_str();

  ESP_Mail_Session session;
  session.server.host_name = SMTP_HOST;
  session.server.port = SMTP_PORT; // 587
  session.login.email = AUTHOR_EMAIL;
  session.login.password = AUTHOR_PASSWORD;

  if (!smtp.connect(&session)) {
    Serial.println("SMTP connection failed");
    return;
  }

  if (MailClient.sendMail(&smtp, &email)) {
    Serial.println("Emergency Email successfully dispatched to patient!");
  } else {
    Serial.print("Email failed: ");
    Serial.println(smtp.errorReason());
  }
  smtp.closeSession();
}