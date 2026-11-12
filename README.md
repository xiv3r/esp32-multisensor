# esp32-multisensor
ESP32 IoT Environmental Monitoring System with multiple sensors.

<image src="https://github.com/xiv3r/esp32-multisensor/blob/main/image.png">

## 1. Hardware Required

| Component | Quantity | Notes |
|---|---|---|
| ESP32 DevKit (WROOM-32) | 1 | Any ESP32 variant with 4MB+ flash |
| DHT22 (AM2302) | 1 | Temperature + humidity |
| MQ2 gas sensor module | 1 | Analog output |
| LDR + 10kΩ resistor | 1 | Light sensor (voltage divider) |
| HC-SR501 PIR | 1 | Motion sensor |
| SSD1306 OLED 128×64 | 1 | I2C, address 0x3C |
| Active buzzer (5V) | 1 | Alarm output |
| Relay module (5V) | 1 | Load control |
| LED + 220Ω resistor | 1 | Status indicator |
| Push button | 1 | Momentary, for mute/reset |
| Breadboard + jumper wires | — | — |

## 2. Wiring

### Power
- ESP32 `3V3` → DHT22 `VCC`, SSD1306 `VCC`
- ESP32 `VIN` (5V from USB) → MQ2 `VCC`, PIR `VCC`, buzzer `+`, relay `VCC`
- All `GND` → common ground

### Sensor connections

| Signal | ESP32 Pin | Component |
|---|---|---|
| DHT22 DATA | GPIO 4 | + 10kΩ pull-up to 3V3 |
| MQ2 AOUT | GPIO 35 | Analog input |
| LDR (divider midpoint) | GPIO 34 | Analog input |
| PIR OUT | GPIO 27 | Digital input |
| OLED SDA | GPIO 21 | I2C data |
| OLED SCL | GPIO 22 | I2C clock |
| Buzzer + | GPIO 26 | Digital output |
| Relay IN | GPIO 25 | Digital output |
| LED anode | GPIO 33 | Digital output |
| Button | GPIO 0 | To GND (internal pull-up) |

⚠️ **Important**: Do **not** hold the button during power-on — GPIO 0 held low at boot enters flash mode. Press only after boot.

⚠️ **Relay polarity**: Most relay modules are **active-LOW**. If yours is, set `RELAY_ACTIVE_HIGH 0` at the top of the code. Same for buzzer (`BUZZER_ACTIVE_HIGH`) and LED (`LED_ACTIVE_HIGH`).

⚠️ **LDR polarity**: If using the **analog** output of an LDR module (bright = high ADC), set the "High ADC Means Dark" option to **No** in the LDR settings page after boot. Verify by covering the sensor.

## 3. Software Setup

### 3.1 Install Arduino IDE + ESP32 core
1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. **File → Preferences → Additional Boards Manager URLs**: add
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. **Tools → Board → Boards Manager** → search "esp32" → install **esp32 by Espressif** (v2.0.14 or v3.x both work).
4. **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
5. Set **Tools → Partition Scheme → Default 4MB with spiffs** (or any with ≥1.2MB app).

### 3.2 Install libraries
**Tools → Manage Libraries**, install:
- **DHT sensor library** (Adafruit) + **Adafruit Unified Sensor**
- **Adafruit SSD1306** + **Adafruit GFX Library**
- **Adafruit BusIO** (auto-installed as dependency)

The following are built into the ESP32 core (no install needed):
`WiFi`, `WebServer`, `DNSServer`, `Preferences`, `Wire`, `esp_task_wdt`, `esp_wifi`, `esp_system`.

### 3.3 Configure the sketch
Open the provided `.ino` file. At the top, verify/change:

```cpp
// Active levels — match your hardware
#define BUZZER_ACTIVE_HIGH   1   // set 0 if buzzer sounds on LOW
#define LED_ACTIVE_HIGH      1   // set 0 if LED lights on LOW
#define RELAY_ACTIVE_HIGH    1   // set 0 for active-LOW relay modules
```

Optional: change the default WiFi credentials and AP channel:
```cpp
#define DEFAULT_WIFI_SSID   "ESP32_Multi_Sensors"
#define DEFAULT_WIFI_PASS   "12345678"       // must be 8–64 chars
#define DEFAULT_AP_CHANNEL  6
```

### 3.4 Upload
1. Connect ESP32 via USB.
2. **Tools → Port** → select the ESP32's port.
3. Click **Upload** (→).
4. If upload fails with "Connecting...", hold the BOOT button during the first few seconds of upload.

## 4. First Boot

1. Open **Tools → Serial Monitor** at **115200 baud** (optional; the code doesn't print, but you can watch for resets).
2. After ~2 seconds, the OLED shows "Warming up…".
3. On your phone or laptop, scan for WiFi networks. Connect to:
   - **SSID**: `ESP32_Multi_Sensors`
   - **Password**: `12345678`
4. A captive portal should pop up automatically. If not, open a browser and go to **http://192.168.4.1**.

**Note**: The device warms up for 30 seconds (MQ2 heater). During this time, sensor values display but alarms are suppressed.

## 5. Using the Web Interface

### Dashboard (`/`)
- Live sensor readings (updates every 2 s)
- System state banner: **Normal / Warm Up / Alarm / Sensor Failure**
- Uptime, free heap, mute status
- Red = alarm condition, orange = warning, green = OK

### Settings (`/settings`)
Per-sensor configuration pages:
- **DHT22**: enable, alarm threshold (e.g., `400` = 40.0°C), calibration offsets
- **MQ2**: enable, threshold (0–4095), calibration offset
- **LDR**: enable, dark threshold, polarity
- **PIR**: enable, motion hold time
- **Buzzer**: enable, mute duration (≥1 s)
- **Relay**: enable, relay hold time
- **System**: warmup, sensor/display intervals, alarm latch time

Each page has a **Save** button. Settings are written to flash within 30 s (debounced) and survive reboot.

### WiFi (`/wifi`)
- Change AP SSID, password, or channel.
- **Saving restarts the AP** — you'll be disconnected. Reconnect to the new SSID.
- Password is masked; you must retype it to change any WiFi setting.

### Factory Reset
- **Web**: Settings → Maintenance → Factory Reset
- **Button**: Hold the button for 5 s (only when **not** in alarm state)

## 6. Normal Operation

### The device is **NORMAL** when:
- Gas level ≤ threshold
- Temperature ≤ threshold
- DHT is reporting valid readings

### Alarm triggers when:
- **Gas**: MQ2 reading exceeds threshold → buzzer sounds, LED lights, relay turns on
- **Temperature**: DHT temp exceeds threshold
- **Alarm remains latched for `alarmLatchTime`** (default 10 s) after the condition clears

### Press the button once during an alarm to **mute** the buzzer (default 60 s). LED stays on.

### **SENSOR_FAIL** state (purple banner + blinking buzzer):
- 4+ consecutive failed DHT reads → indicates disconnected/faulty DHT
- Mute is disabled in this state
- Check DHT wiring and 10kΩ pull-up

### **Relay behavior**:
- Turns ON when motion is detected **and** it's dark (per LDR)
- Turns ON when gas alarm is active
- Stays ON for `max(pirHoldTime, relayHoldTime)` after last motion (default 30 s each)

## 7. Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| OLED blank | Check I2C wiring (SDA→21, SCL→22, VCC, GND). Try address `0x3D` if `0x3C` fails. |
| DHT reads fail (SENSOR_FAIL) | Missing 10kΩ pull-up on DATA, or bad wire. Swap sensor to test. |
| Gas alarm immediately at boot | Warmup too short — increase `warmupTime` in System settings to 60–90 s. Sensor needs burn-in for first use (24–48 h on first power). |
| Relay is ON when it should be OFF | Wrong polarity — set `RELAY_ACTIVE_HIGH 0` and re-upload. |
| Buzzer sounds continuously | Gas above threshold or temp alarm. Check settings. Press button to mute. |
| Can't connect to AP | Check channel (use 1, 6, or 11). Try channel 6. Power-cycle ESP32. |
| Web pages don't load | Reconnect to ESP32's WiFi. Try `http://192.168.4.1` directly. |
| Changes not saved after reboot | NVS might be failing — check Serial for `nvs` errors. Factory reset and retry. |
| Upload fails | Hold BOOT during upload. Install CP210x/CH340 USB drivers if port not detected. |
| Config reset after firmware update | Known behavior: `CONFIG_VERSION` bump wipes NVS. Reconfigure via web. |

## 8. Maintenance

- **NVS lifespan**: settings are written with a 30 s debounce, so rapid saves don't wear flash. Typical lifetime: millions of writes.
- **Warmup**: after power cycle, wait 30 s (or configured time) before trusting alarms.
- **MQ2 burn-in**: first-ever use requires 24–48 hours powered to stabilize readings.
- **Firmware update**: recompile with `#define CONFIG_VERSION 3` if you add fields to `ConfigData` (this will trigger one factory reset).
- **Logging**: enable `Serial.begin(115200)` in `setup()` if you want to debug.

## 9. Configuration Reference

### Default credentials
- SSID: `ESP32_Multi_Sensors` · Password: `12345678` · IP: `192.168.4.1` · Channel: 6

### Default thresholds
| Setting | Default | Valid range |
|---|---|---|
| Gas threshold | 2000 | 0–4095 |
| Temp threshold | 400 (40.0 °C) | −40.0 to 125.0 °C |
| LDR dark threshold | 1500 | 0–4095 |
| Warmup | 30 s | 0–600 s |
| Sensor interval | 2 s | ≥2 s (DHT on) |
| Display interval | 500 ms | 100 ms–10 s |
| Alarm latch | 10 s | 0–1 h |
| Mute duration | 60 s | ≥1 s |
| PIR/Relay hold | 30 s each | 0–1 h |

### Calibration tips
- **DHT temperature**: compare to a reference thermometer; adjust `dhtCalibTempX10` (units of 0.1 °C).
- **DHT humidity**: same, with `dhtCalibHumX10` (units of 0.1 %).
- **MQ2**: expose to clean air, note reading, set `gasCalibOffset` to force baseline to 0. Then set threshold ~30% below typical alarm concentration.
- **LDR**: cover sensor, note ADC value → set `ldrDarkThreshold` slightly above that value (if `ldrHighMeansDark=1`).

## 10. Safety Notes

- This is a **monitoring** device, not a certified safety system. Do not rely on it as the sole alarm for gas, fire, or intrusion.
- **MQ2 measures combustible gas** but is not calibrated to a specific ppm. Treat thresholds as relative, not absolute.
- **Relay can switch mains voltage** on some modules — ensure proper isolation and grounding. Consult an electrician.
- **Alarm muting** keeps the LED on as a visual reminder. Never leave the device unattended during a gas event.
- **AP password** — change from default before deployment.
