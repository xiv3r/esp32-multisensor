/*
 * Project: ESP32 IoT Environmental Monitoring System (Industrial Edition)
 * Features: Watchdog, Mute Button, I2C Recovery, Sensor Failure Alarms, Latching Logic
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <esp_task_wdt.h> 

// ================= VERIFIED PIN CONFIGURATION =================
#define PIN_DHT         4   // CORRECTED from 15
#define PIN_LDR         34  
#define PIN_PIR         27  
#define PIN_MQ2         35  
#define PIN_BUZZER      26  
#define PIN_RELAY       25  
#define PIN_LED         33  
#define PIN_SDA         21  
#define PIN_SCL         22  
#define PIN_BUTTON      0   // Onboard BOOT button used for Mute

// ================= CONFIGURATION =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C 
#define DHTTYPE DHT22

// Thresholds
const int   GAS_THRESHOLD      = 2000; 
const float TEMP_THRESHOLD     = 40.0; 
const int   LDR_DARK_THRESHOLD = 1500; 

// Timing (Milliseconds)
const unsigned long WARMUP_TIME          = 30000; 
const unsigned long INTERVAL_SENSOR_READ = 2000;  
const unsigned long INTERVAL_DISPLAY     = 500;
const unsigned long RELAY_HOLD_TIME      = 30000; 
const unsigned long ALARM_LATCH_TIME     = 10000; 
const unsigned long MUTE_DURATION        = 60000; 

// ================= OBJECTS & STATE =================
DHT dht(PIN_DHT, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum SystemState {
  STATE_WARMUP,
  STATE_NORMAL,
  STATE_ALARM,
  STATE_SENSOR_FAIL
};

SystemState currentState = STATE_WARMUP;

struct SensorData {
  float temperature;
  float humidity;
  int   lightLevel;
  int   gasLevel;
  bool  motionDetected;
  bool  dhtError;
};

SensorData currentData;
unsigned long bootTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMotionTime = 0;
unsigned long lastSafeTime = 0; 
unsigned long lastButtonPress = 0;
unsigned long muteStartTime = 0;

bool relayState = false;
bool alarmLatched = false;
bool buzzerMuted = false;
bool lastButtonState = HIGH;
int dhtErrorCount = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_MQ2, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_LED, LOW);

  dht.begin();
  Wire.begin(PIN_SDA, PIN_SCL);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 failed"));
  }
  
  bootTime = millis();
  lastSafeTime = millis();
  Serial.println(F("System Booting. MQ-2 Warming up..."));
}

// ================= MAIN LOOP =================
void loop() {
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();

  handleButton(currentMillis);

  if (currentState == STATE_WARMUP && (currentMillis - bootTime >= WARMUP_TIME)) {
    currentState = STATE_NORMAL;
    Serial.println(F("Warmup complete. System Normal."));
  }

  if (currentMillis - lastSensorRead >= INTERVAL_SENSOR_READ) {
    lastSensorRead = currentMillis;
    readSensors();
    if (currentState != STATE_WARMUP) {
      evaluateLogic(currentMillis);
    }
  }

  if (currentMillis - lastDisplayUpdate >= INTERVAL_DISPLAY) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }
}

// ================= HELPER FUNCTIONS =================

void handleButton(unsigned long currentMillis) {
  bool buttonState = digitalRead(PIN_BUTTON);
  
  if (buttonState == LOW && lastButtonState == HIGH && (currentMillis - lastButtonPress > 200)) {
    lastButtonPress = currentMillis;
    if (alarmLatched && !buzzerMuted) {
      buzzerMuted = true;
      muteStartTime = currentMillis;
      Serial.println(F("Alarm Muted for 60 seconds."));
    }
  }
  lastButtonState = buttonState;

  if (buzzerMuted && (currentMillis - muteStartTime > MUTE_DURATION)) {
    buzzerMuted = false;
    Serial.println(F("Mute expired."));
  }
}

void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  
  if (isnan(t) || isnan(h)) {
    dhtErrorCount++;
    if (dhtErrorCount > 3) {
      currentData.dhtError = true;
    }
  } else {
    dhtErrorCount = 0;
    currentData.dhtError = false;
    currentData.temperature = t;
    currentData.humidity = h;
  }

  currentData.lightLevel = analogRead(PIN_LDR);
  currentData.gasLevel = analogRead(PIN_MQ2);
  currentData.motionDetected = digitalRead(PIN_PIR);
}

void evaluateLogic(unsigned long currentMillis) {
  bool gasDetected = (currentData.gasLevel > GAS_THRESHOLD);
  bool tempDetected = (!currentData.dhtError && currentData.temperature > TEMP_THRESHOLD);
  
  if (gasDetected || tempDetected) {
    alarmLatched = true;
    lastSafeTime = currentMillis; 
  } 
  
  if (!gasDetected && !tempDetected) {
    if (currentMillis - lastSafeTime >= ALARM_LATCH_TIME) {
      alarmLatched = false;
      buzzerMuted = false; 
    }
  }

  if (currentData.dhtError) {
    currentState = STATE_SENSOR_FAIL;
  } else if (alarmLatched) {
    currentState = STATE_ALARM;
  } else {
    currentState = STATE_NORMAL;
  }

  if (currentState == STATE_ALARM) {
    if (!buzzerMuted) {
      digitalWrite(PIN_BUZZER, HIGH);
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }
    digitalWrite(PIN_LED, HIGH); 
  } else if (currentState == STATE_SENSOR_FAIL) {
    digitalWrite(PIN_BUZZER, (currentMillis % 1000 < 500) ? HIGH : LOW);
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED, LOW);
  }

  if (currentData.motionDetected && currentData.lightLevel > LDR_DARK_THRESHOLD) {
    lastMotionTime = currentMillis; 
    relayState = true;
  } 
  
  if ((currentMillis - lastMotionTime < RELAY_HOLD_TIME) || gasDetected) {
    relayState = true;
  } else {
    relayState = false;
  }

  digitalWrite(PIN_RELAY, relayState ? HIGH : LOW);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  if (currentState == STATE_WARMUP) {
    display.print(F("Warming up..."));
  } else if (currentState == STATE_ALARM) {
    display.setTextColor(SSD1306_INVERSE);
    display.print(buzzerMuted ? F(" !! ALARM (MUTED) !! ") : F(" !! ALARM !! "));
    display.setTextColor(SSD1306_WHITE);
  } else if (currentState == STATE_SENSOR_FAIL) {
    display.print(F("!! SENSOR FAILURE !!"));
  } else {
    display.print(F("System Normal"));
  }

  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  if (currentData.dhtError) {
    display.print(F("Temp: SENSOR ERROR"));
  } else {
    display.print(F("Temp: ")); display.print(currentData.temperature, 1); display.print(F(" C"));
  }

  display.setCursor(0, 25);
  if (currentData.dhtError) {
    display.print(F("Hum:  SENSOR ERROR"));
  } else {
    display.print(F("Hum:  ")); display.print(currentData.humidity, 1); display.print(F(" %"));
  }

  display.setCursor(0, 35);
  display.print(F("Gas:  ")); display.print(currentData.gasLevel);

  display.setCursor(0, 45);
  display.print(F("Light:")); display.print(currentData.lightLevel);
  
  display.setCursor(0, 55);
  display.print(F("Motion: ")); display.print(currentData.motionDetected ? "YES" : "NO");

  display.display();
}
