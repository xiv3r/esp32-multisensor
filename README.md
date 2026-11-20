# esp32-multisensor
ESP32 IoT Environmental Monitoring System with multiple sensors.

<image src="https://github.com/xiv3r/esp32-multisensor/blob/main/image.png">

## What This Is
An ESP32 (classic) firmware sketch that turns the board into a standalone environmental monitoring node with:
- **DHT22** (temp/humidity), **MQ-2** (gas), **LDR** (light), **PIR** (motion)
- **SSD1306 OLED** (I²C) status display
- **Buzzer, LED, and 5 relay outputs** driven by alarm rules
- **Wi-Fi access point** at `192.168.4.1` with an embedded web console
- **Persistent config** stored in NVS with dual-bank CRC-verified records
- **Factory reset** via 5-second BOOT button hold

## Required Libraries (install via Arduino Library Manager)
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `DHT sensor library` (Adafruit)
- ESP32 board package (includes `Preferences`, `WiFi`, `Wire`, ADC oneshot driver, etc.)

## Board Settings
- **Board:** ESP32 Dev Module (classic ESP32 — **not** S2/S3/C3)
- **Core:** must run Arduino loop on **Core 1** (the sketch `static_assert`s this)
- **Partition scheme:** default is fine (NVS required)

## Wiring Map (fixed — do not change)
| Function | GPIO |
|---|---|
| DHT22 data | 4 |
| MQ-2 analog | 35 (ADC1) |
| LDR analog | 34 (ADC1) |
| PIR signal | 27 |
| BOOT button | 0 (internal pull-up) |
| OLED SDA / SCL | 21 / 22 |
| Relay unified | 25 |
| Relay DHT / Gas / Light / PIR | 15 / 18 / 19 / 5 |
| Buzzer | 26 |
| LED | 33 |

⚠️ **GPIO 5 and 15 are boot-strapping pins** — use high-impedance drivers and a hardware inhibit if outputs must stay off during boot. Never drive relay coils directly from GPIOs.

## First Boot
1. Flash the sketch and open Serial Monitor (any baud; logging is disabled).
2. On a phone/PC, join Wi-Fi **`ESP32_Multi_Sensors`** with password **`12345678`**.
3. Open **http://192.168.4.1/** in a browser.
4. The dashboard shows live readings; use the sidebar to configure each module.

## Configuration Notes
- All settings are range-checked; hysteresis must keep the clear point inside the sensor range.
- **Save cooldown:** 15 s between writes, max 12 writes per hour (flash wear protection).
- **Wi-Fi changes** save first, then reboot ~10 s later — reconnect to the new SSID.
- **DHT22** minimum sample interval is 2.5 s; a failed/stale sample clears its alarms.
- **MQ-2** has a warm-up delay (default 180 s) during which readings are withheld.
- **PIR hold** starts after the last debounced HIGH.

## Factory Reset
Hold the **BOOT (GPIO 0)** button for **≥5 s** while running, then release. Outputs turn off, defaults are written, and the device restarts. Reconnect with the default SSID/password.

## Safety Reminders
- MQ-2 output is **raw ADC counts, not ppm** — this is **not** a certified gas/smoke alarm.
- Use a voltage divider so analog inputs never exceed 3.3 V.
- The AP is **WPA2, local-only**; anyone on it can manage the device — change the password before deployment.
- All outputs are **fail-off** during startup, storage errors, or reset.

## Troubleshooting
| Symptom | Check |
|---|---|
| OLED blank | I²C address (0x3C/0x3D), SDA=21/SCL=22, module powered |
| DHT shows `--` | Pull-up to 3.3 V, GPIO 4, sample interval ≥2500 ms |
| ADC reads implausible | Divider values, common ground, GPIO 34/35 are input-only |
| "Storage error" banner | NVS full/corrupt — factory reset, then re-save |
| Can't reach 192.168.4.1 | Rejoin AP; captive portal may need manual URL |
| Save rejected with 429 | Wait for cooldown shown in **System status** |
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
