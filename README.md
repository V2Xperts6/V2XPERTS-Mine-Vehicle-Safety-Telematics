# V2XPERTS-Mine-Vehicle-Safety-Telematics
AI-powered mine vehicle safety and telematics system with multi-sensor fusion, predictive collision alerts, geo-fencing, and real-time cloud monitoring.
/*
  ESP32 Collision Warning + Voice Alerts + Blynk Motor Control (final)

  - HC-SR04 measures distance, time-to-collision (TTC) is calculated from closing speed
  - LEDs, buzzer and 16x2 I2C LCD show SAFE / CAUTION / DANGER
  - MAX98357A speaks the distance over WiFi (Google text-to-speech)
  - Blynk buttons on V1..V4 drive the motors (Forward, Backward, Left, Right; Push mode, 0-1)

  Speed: the sensor, LCD and safety logic run in loop() on core 1.
  Audio, WiFi text-to-speech and Blynk run in their own task on core 0, so slow
  network calls can no longer freeze the TTC readings.

  Safety:
    - Forward is stopped and blocked in DANGER; backward, left, right still work
    - Motors stop if Blynk disconnects
    - Motors run until you stop them or DANGER is detected (no time limit by default)
    - Optional auto-stop: set MOTOR_MAX_RUN_MS above 0 (milliseconds); 0 = off

  Libraries: LiquidCrystal I2C (Frank de Brabander), ESP32-audioI2S (schreibfaul1), Blynk (Blynk)
  Board: ESP32 Dev Module, Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
*/

// ===== BLYNK CREDENTIALS (copy from Blynk: Device Info > Firmware Configuration) =====
#define BLYNK_TEMPLATE_ID   "TMPL3YmlWzYer"
#define BLYNK_TEMPLATE_NAME "V2Xperts "
#define BLYNK_AUTH_TOKEN    "90B-3P7cBm-O_v9CypNGCt0k2UR0_zLk"
// ============================================================================
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "Audio.h"

// ---------------- Pin definitions ----------------
#define TRIG_PIN    5
#define ECHO_PIN    18      // through a 1k/2k voltage divider
#define LED_GREEN   25
#define LED_YELLOW  26
#define LED_RED     27
#define BUZZER      4

// I2S audio (MAX98357A)
#define I2S_BCLK    23
#define I2S_LRC     19
#define I2S_DOUT    33

// Motor driver inputs (L298N)
#define IN1 13  // Left motor forward
#define IN2 16  // Left motor backward
#define IN3 14  // Right motor forward
#define IN4 32  // Right motor backward

// ---------------- WiFi & Blynk ----------------
char ssid[] = "V2xperts";
char pass[] = "jai@1234";
char auth[] = BLYNK_AUTH_TOKEN;
const unsigned long WIFI_TIMEOUT_MS = 15000;
const unsigned long BLYNK_RETRY_MS  = 30000;

// ---------------- Settings ----------------
const unsigned long SENSOR_INTERVAL_MS        = 80;     // HC-SR04 needs at least ~60 ms
const unsigned long SPEECH_INTERVAL_MS        = 5000;
const unsigned long SPEECH_INTERVAL_DANGER_MS = 3000;
const unsigned long MOTOR_MAX_RUN_MS          = 0;      // 0 = no time limit

const float SMOOTHING_ALPHA       = 0.5;    // higher = faster reaction, lower = smoother
const float SPEED_DEADBAND_CMPS   = 5.0;    // lower = TTC shows sooner, but noisier
const float TTC_SAFE_THRESHOLD    = 3.0;
const float TTC_CAUTION_THRESHOLD = 1.5;
const float ABSOLUTE_DANGER_CM    = 50.0;
const float LOST_ECHO_NEAR_CM     = 60.0;

const int AUDIO_VOLUME = 15;   // 0..21

LiquidCrystal_I2C lcd(0x27, 16, 2);   // use 0x3F if the LCD stays blank
Audio audio;

// ---------------- State ----------------
unsigned long prevSensorTime = 0;
unsigned long lastSpeechTime = 0;
unsigned long lastBlynkTryMs = 0;
float prevDistanceCm = -1;
float lastValidCm = -1;
float smoothedSpeed = 0;

volatile bool emergencyStop = false;            // blocks forward movement when too close
volatile int  motorDir = 0;                     // 0 stop, 1 forward, 2 backward, 3 left, 4 right
volatile unsigned long motorStartMs = 0;
volatile bool audioBusy = false;
volatile bool resetForwardSwitch = false;       // tells Blynk to flip the Forward switch back to OFF

QueueHandle_t speechQueue;

// ---------------- Motor control ----------------
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  motorDir = 0;
}

void moveForward() {
  if (emergencyStop) {
    resetForwardSwitch = true;   // forward is blocked in DANGER
    return;
  }
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorDir = 1;
  motorStartMs = millis();
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  motorDir = 2;
  motorStartMs = millis();
}

void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorDir = 3;
  motorStartMs = millis();
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  motorDir = 4;
  motorStartMs = millis();
}

void stopIfMovingForward() {
  if (motorDir == 1) {
    stopMotors();
    resetForwardSwitch = true;   // turn the Forward switch in the app back to OFF
  }
}

// ---------------- Blynk buttons (Push mode, Integer 0-1) ----------------
BLYNK_WRITE(V1) {   // Forward
  if (param.asInt() == 1) moveForward();
  else stopMotors();
}

BLYNK_WRITE(V2) {   // Backward (always allowed)
  if (param.asInt() == 1) moveBackward();
  else stopMotors();
}

BLYNK_WRITE(V3) {   // Left
  if (param.asInt() == 1) turnLeft();
  else stopMotors();
}

BLYNK_WRITE(V4) {   // Right
  if (param.asInt() == 1) turnRight();
  else stopMotors();
}

void handleBlynk() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (Blynk.connected()) {
    Blynk.run();
    if (resetForwardSwitch) {
      resetForwardSwitch = false;
      Blynk.virtualWrite(V1, 0);
    }
  } else if (!audioBusy && millis() - lastBlynkTryMs > BLYNK_RETRY_MS) {   // never retry while speaking
    lastBlynkTryMs = millis();
    Blynk.connect(2000);
  }
}

// ---------------- Sensor & display helpers ----------------
float getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

void setLEDs(bool g, bool y, bool r) {
  digitalWrite(LED_GREEN, g);
  digitalWrite(LED_YELLOW, y);
  digitalWrite(LED_RED, r);
}

// Only rewrites a line when its text changed, which keeps I2C traffic (and delay) low
char lastLine1[17] = "";
char lastLine2[17] = "";

void showLcd(const char* line1, const char* line2) {
  char a[17], b[17];
  snprintf(a, sizeof(a), "%-16s", line1);
  snprintf(b, sizeof(b), "%-16s", line2);
  if (strcmp(a, lastLine1) != 0) {
    lcd.setCursor(0, 0); lcd.print(a);
    strcpy(lastLine1, a);
  }
  if (strcmp(b, lastLine2) != 0) {
    lcd.setCursor(0, 1); lcd.print(b);
    strcpy(lastLine2, b);
  }
}

// Sends the text to the audio task; it never waits for the network
void maybeSpeak(const char* text, unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastSpeechTime < intervalMs) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (audioBusy) return;

  char buf[64];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  xQueueOverwrite(speechQueue, buf);
  lastSpeechTime = now;
}

// ---------------- Network / audio task (core 0) ----------------
void networkTask(void* param) {
  char buf[64];
  for (;;) {
    audio.loop();

    if (xQueueReceive(speechQueue, buf, 0) == pdTRUE) {
      if (!audio.isRunning()) {
        Serial.printf("Speaking: %s\n", buf);
        audio.connecttospeech(buf, "en");
      }
    }
    audioBusy = audio.isRunning();

    handleBlynk();
    vTaskDelay(1);
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  setLEDs(false, false, false);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Wire.begin(21, 22);
  lcd.init(); lcd.backlight();
  Wire.setClock(400000);   // faster LCD updates
  showLcd("Connecting WiFi", "");

  Blynk.config(auth);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);
  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_TIMEOUT_MS) {
    delay(500); Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());
    showLcd("WiFi connected!", "Connecting Blynk");

    if (Blynk.connect(5000)) {
      Serial.println("Blynk connected");
      showLcd("Blynk connected!", "Voice alerts ON");
    } else {
      Serial.println("Blynk not connected (check the three Blynk lines at the top)");
      showLcd("Blynk failed", "Will retry");
    }
  } else {
    Serial.println("WiFi not connected. Voice alerts & app disabled.");
    showLcd("No WiFi", "Offline Mode");
  }

  delay(1200);
  showLcd("", "");

  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(AUDIO_VOLUME);

  speechQueue = xQueueCreate(1, 64);
  xTaskCreatePinnedToCore(networkTask, "network", 12288, NULL, 1, NULL, 0);

  // Startup phrase: lets you test the speaker without the ultrasonic sensor
  if (WiFi.status() == WL_CONNECTED) {
    char hello[64] = "System ready";
    xQueueOverwrite(speechQueue, hello);
    lastSpeechTime = millis();
  } else {
    Serial.println("No WiFi, so the voice is disabled");
  }

  prevSensorTime = millis();
}

// ---------------- Main loop: sensor, TTC, LCD, alerts (core 1) ----------------
void loop() {
  // Failsafes: stop if the app connection is lost or a button release never arrived
  if (motorDir != 0) {
    if (!Blynk.connected() || (MOTOR_MAX_RUN_MS > 0 && millis() - motorStartMs > MOTOR_MAX_RUN_MS)) {
      stopMotors();
    }
  }

  unsigned long nowMs = millis();
  if (nowMs - prevSensorTime < SENSOR_INTERVAL_MS) return;

  float dtSec = (nowMs - prevSensorTime) / 1000.0f;
  prevSensorTime = nowMs;

  float distanceCm = getDistanceCm();

  // ----- No echo received -----
  if (distanceCm < 0) {
    prevDistanceCm = -1;
    smoothedSpeed = 0;

    if (lastValidCm >= 0 && lastValidCm < LOST_ECHO_NEAR_CM) {
      emergencyStop = true;
      stopIfMovingForward();
      setLEDs(false, false, true);
      tone(BUZZER, 2000);
      showLcd("Echo lost (near)", "Status: DANGER!");
      maybeSpeak("Danger. Obstacle too close", SPEECH_INTERVAL_DANGER_MS);
    } else {
      emergencyStop = false;
      setLEDs(false, false, false);
      noTone(BUZZER);
      showLcd("Dist:---  TTC:--", "Status: NO SIG");
    }
    return;
  }

  lastValidCm = distanceCm;

  // ----- Closing speed and time-to-collision -----
  float speedCmPerSec = 0;
  float ttcSec = -1;

  if (prevDistanceCm >= 0 && dtSec > 0) {
    float rawSpeed = (prevDistanceCm - distanceCm) / dtSec;
    smoothedSpeed = (SMOOTHING_ALPHA * rawSpeed) + ((1.0f - SMOOTHING_ALPHA) * smoothedSpeed);
    speedCmPerSec = smoothedSpeed;

    if (speedCmPerSec > SPEED_DEADBAND_CMPS) {
      ttcSec = distanceCm / speedCmPerSec;
    }
  }

  Serial.printf("Distance: %.1f cm | Speed: %.1f cm/s | Motor: %d\n", distanceCm, speedCmPerSec, (int)motorDir);

  // ----- LCD line 1 -----
  char ttcText[10];
  if (ttcSec >= 0 && ttcSec < 99) snprintf(ttcText, sizeof(ttcText), "%.1fs", ttcSec);
  else snprintf(ttcText, sizeof(ttcText), "--s");

  char line1[24];
  snprintf(line1, sizeof(line1), "D:%.0fcm T:%s", distanceCm, ttcText);

  // ----- Status, LEDs, buzzer, LCD line 2 and voice -----
  const char* status;
  char speech[64];
  int cm = (int)distanceCm;

  if (distanceCm < ABSOLUTE_DANGER_CM || (ttcSec >= 0 && ttcSec < TTC_CAUTION_THRESHOLD)) {
    emergencyStop = true;
    stopIfMovingForward();
    setLEDs(false, false, true);
    tone(BUZZER, 2000);
    status = "Status: DANGER!";
    snprintf(speech, sizeof(speech), "Danger. Distance is %d centimeters", cm);
    maybeSpeak(speech, SPEECH_INTERVAL_DANGER_MS);
  } else if (ttcSec >= 0 && ttcSec < TTC_SAFE_THRESHOLD) {
    emergencyStop = false;
    setLEDs(false, true, false);
    tone(BUZZER, 1000, 150);
    status = "Status: CAUTION";
    snprintf(speech, sizeof(speech), "Caution. Distance is %d centimeters", cm);
    maybeSpeak(speech, SPEECH_INTERVAL_MS);
  } else {
    emergencyStop = false;
    setLEDs(true, false, false);
    noTone(BUZZER);
    status = "Status: SAFE";
    snprintf(speech, sizeof(speech), "Distance is %d centimeters", cm);
    maybeSpeak(speech, SPEECH_INTERVAL_MS);
  }

  showLcd(line1, status);
  prevDistanceCm = distanceCm;
}
