/*
 * ESP32 IoT Environmental Monitoring System - Industrial Edition
 * Full Web UI, Per-Sensor Settings Pages, Captive Portal, NVS Optimization, Non-Blocking Architecture
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_system.h>

// ================= PIN CONFIGURATION =================
#define PIN_DHT         4
#define PIN_LDR         34
#define PIN_PIR         27
#define PIN_MQ2         35
#define PIN_BUZZER      26
#define PIN_RELAY       25
#define PIN_LED         33
#define PIN_SDA         21
#define PIN_SCL         22
#define PIN_BUTTON      0

// ================= HARDWARE CONFIG =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
#define DHTTYPE DHT22

// ================= DEFAULTS =================
#define DEFAULT_GAS_THRESHOLD      2000
#define DEFAULT_TEMP_THRESHOLD     400
#define DEFAULT_LDR_DARK_THRESHOLD 1500
#define DEFAULT_WARMUP_TIME        30000UL
#define DEFAULT_SENSOR_INTERVAL    2000UL
#define DEFAULT_DISPLAY_INTERVAL   500UL
#define DEFAULT_RELAY_HOLD_TIME    30000UL
#define DEFAULT_PIR_HOLD_TIME      30000UL
#define DEFAULT_ALARM_LATCH_TIME   10000UL
#define DEFAULT_MUTE_DURATION      60000UL
#define DEFAULT_AP_CHANNEL         6
#define DEFAULT_WIFI_SSID          "ESP32_Multi_Sensors"
#define DEFAULT_WIFI_PASS          "12345678"

// ================= ACTIVE-LEVEL =================
#define BUZZER_ACTIVE_HIGH   1
#define LED_ACTIVE_HIGH      1
#define RELAY_ACTIVE_HIGH    1
#define BUZZER_ON()  digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? HIGH : LOW)
#define BUZZER_OFF() digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? LOW : HIGH)
#define LED_ON()     digitalWrite(PIN_LED,    LED_ACTIVE_HIGH    ? HIGH : LOW)
#define LED_OFF()    digitalWrite(PIN_LED,    LED_ACTIVE_HIGH    ? LOW : HIGH)
#define RELAY_ON()   digitalWrite(PIN_RELAY,  RELAY_ACTIVE_HIGH  ? HIGH : LOW)
#define RELAY_OFF()  digitalWrite(PIN_RELAY,  RELAY_ACTIVE_HIGH  ? LOW : HIGH)

// ================= CONSTANTS =================
#define FACTORY_RESET_HOLD_MS      5000UL
#define WDT_TIMEOUT_MS             10000UL
#define BUTTON_DEBOUNCE_MS         200UL
#define DNS_PORT                   53
#define NVS_SAVE_MIN_INTERVAL_MS   30000UL
#define CONFIG_MAGIC               0x45535032UL
#define CONFIG_VERSION             2

#define ELAPSED(now, since)  ((uint32_t)((now) - (since)))

// ================= OBJECTS =================
DHT dht(PIN_DHT, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// ================= ENUMS =================
enum SystemState : uint8_t {
  STATE_WARMUP = 0,
  STATE_NORMAL,
  STATE_ALARM,
  STATE_SENSOR_FAIL
};

// ================= SENSOR DATA =================
struct SensorData_t {
  float    temperature;
  float    humidity;
  int64_t  lightLevel;
  int64_t  gasLevel;
  bool     motionDetected;
  bool     dhtError;
};

SensorData_t currentData;

// ================= CONFIG =================
struct __attribute__((packed)) ConfigData {
  uint32_t magic;
  uint32_t version;
  int32_t  tempThresholdX10;
  int32_t  dhtCalibTempX10;
  int32_t  dhtCalibHumX10;
  uint8_t  dhtEnabled;
  uint8_t  dhtAlarmEnabled;
  int32_t  gasThreshold;
  int32_t  gasCalibOffset;
  uint8_t  mq2Enabled;
  uint8_t  mq2AlarmEnabled;
  int32_t  ldrDarkThreshold;
  uint8_t  ldrHighMeansDark;
  uint8_t  ldrEnabled;
  uint32_t pirHoldTime;
  uint8_t  pirEnabled;
  uint32_t muteDuration;
  uint8_t  buzzerEnabled;
  uint32_t relayHoldTime;
  uint8_t  relayEnabled;
  uint32_t warmupTime;
  uint32_t sensorInterval;
  uint32_t displayInterval;
  uint32_t alarmLatchTime;
  uint8_t  apChannel;
  char     wifiSSID[33];
  char     wifiPass[65];
  uint32_t crc;
};

// ================= GLOBAL STATE =================
ConfigData config;
SystemState currentState = STATE_WARMUP;

uint32_t bootTime = 0;
uint32_t lastSensorRead = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastMotionTime = 0;
uint32_t lastSafeTime = 0;
uint32_t lastButtonPress = 0;
uint32_t muteStartTime = 0;
uint32_t lastFailBlinkToggle = 0;
uint32_t lastNvsSave = 0;
uint32_t buttonHoldStart = 0;
uint32_t lastDhtRead = 0;

bool relayState = false;
bool alarmLatched = false;
bool buzzerMuted = false;
bool lastButtonState = HIGH;
bool failBlinkState = false;
bool motionLatch = false;
bool configDirty = false;
bool factoryResetPending = false;
bool restartApPending = false;
uint32_t restartApAt = 0;
bool shuttingDown = false;
uint32_t dhtErrorCount = 0;

char jsonBuffer[1024];

// ================= CRC32 =================
uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (~((crc & 1) - 1)));
    }
  }
  return ~crc;
}

uint32_t calculateConfigCRC(const ConfigData* cfg) {
  return calculateCRC32((const uint8_t*)cfg, offsetof(ConfigData, crc));
}

// ================= CONFIG =================
void loadDefaultConfig() {
  memset(&config, 0, sizeof(ConfigData));
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;

  config.tempThresholdX10 = DEFAULT_TEMP_THRESHOLD;
  config.dhtCalibTempX10 = 0;
  config.dhtCalibHumX10 = 0;
  config.dhtEnabled = 1;
  config.dhtAlarmEnabled = 1;

  config.gasThreshold = DEFAULT_GAS_THRESHOLD;
  config.gasCalibOffset = 0;
  config.mq2Enabled = 1;
  config.mq2AlarmEnabled = 1;

  config.ldrDarkThreshold = DEFAULT_LDR_DARK_THRESHOLD;
  config.ldrHighMeansDark = 1;
  config.ldrEnabled = 1;

  config.pirHoldTime = DEFAULT_PIR_HOLD_TIME;
  config.pirEnabled = 1;

  config.muteDuration = DEFAULT_MUTE_DURATION;
  config.buzzerEnabled = 1;

  config.relayHoldTime = DEFAULT_RELAY_HOLD_TIME;
  config.relayEnabled = 1;

  config.warmupTime = DEFAULT_WARMUP_TIME;
  config.sensorInterval = DEFAULT_SENSOR_INTERVAL;
  config.displayInterval = DEFAULT_DISPLAY_INTERVAL;
  config.alarmLatchTime = DEFAULT_ALARM_LATCH_TIME;

  config.apChannel = DEFAULT_AP_CHANNEL;
  strncpy(config.wifiSSID, DEFAULT_WIFI_SSID, sizeof(config.wifiSSID) - 1);
  strncpy(config.wifiPass, DEFAULT_WIFI_PASS, sizeof(config.wifiPass) - 1);

  config.crc = calculateConfigCRC(&config);
}

bool saveConfig() {
  config.crc = calculateConfigCRC(&config);
  preferences.begin("iot_cfg", false);
  size_t written = preferences.putBytes("cfg", &config, sizeof(ConfigData));
  preferences.end();
  if (written == sizeof(ConfigData)) {
    configDirty = false;
    lastNvsSave = millis();
    return true;
  }
  lastNvsSave = millis();
  return false;
}

bool loadConfig() {
  preferences.begin("iot_cfg", true);
  size_t len = preferences.getBytesLength("cfg");
  bool ok = false;
  if (len == sizeof(ConfigData)) {
    ConfigData tmp;
    if (preferences.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (tmp.magic == CONFIG_MAGIC && tmp.version == CONFIG_VERSION &&
          tmp.crc == calculateConfigCRC(&tmp)) {
        memcpy(&config, &tmp, sizeof(ConfigData));
        ok = true;
      }
    }
  }
  preferences.end();
  if (!ok) {
    loadDefaultConfig();
    saveConfig();
  }
  return ok;
}

// ================= I2C RECOVERY =================
void recoverI2C() {
  Wire.end();
  pinMode(PIN_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL);
}

// ================= HTML HEAD/NAV =================
const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP32 Monitor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#c9d1d9;min-height:100vh;padding:12px}
.nav{display:flex;gap:8px;margin-bottom:16px;flex-wrap:wrap}
.nav a{flex:1;min-width:100px;padding:12px;background:#161b22;color:#58a6ff;text-decoration:none;border-radius:8px;text-align:center;font-weight:600;border:1px solid #30363d;transition:all .2s}
.nav a:hover,.nav a.active{background:#1f6feb;color:#fff;border-color:#1f6feb}
.card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:16px;margin-bottom:12px}
.card h2{font-size:15px;color:#8b949e;margin-bottom:12px;text-transform:uppercase;letter-spacing:.5px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
.stat{background:#0d1117;border-radius:8px;padding:14px;text-align:center;border:1px solid #21262d}
.stat .lbl{font-size:11px;color:#8b949e;text-transform:uppercase}
.stat .val{font-size:22px;font-weight:700;color:#58a6ff;margin-top:4px}
.stat.warn .val{color:#f0883e}
.stat.alarm .val{color:#f85149}
.stat.ok .val{color:#3fb950}
.state{padding:10px;border-radius:8px;text-align:center;font-weight:700;margin-bottom:12px;font-size:14px}
.state.normal{background:#238636;color:#fff}
.state.alarm{background:#da3633;color:#fff;animation:pulse 1s infinite}
.state.warmup{background:#9e6a03;color:#fff}
.state.fail{background:#6e40c9;color:#fff}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.65}}
.form-group{margin-bottom:14px}
.form-group label{display:block;font-size:13px;color:#8b949e;margin-bottom:6px}
.form-group input,.form-group select{width:100%;padding:10px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#c9d1d9;font-size:15px}
.form-group input:focus,.form-group select:focus{outline:none;border-color:#1f6feb}
.btn{width:100%;padding:12px;background:#238636;color:#fff;border:none;border-radius:8px;font-size:15px;font-weight:600;cursor:pointer;margin-top:8px;display:block;text-decoration:none;text-align:center}
.btn:hover{background:#2ea043}
.btn.danger{background:#da3633}
.btn.danger:hover{background:#f85149}
.btn.secondary{background:#30363d}
.btn.secondary:hover{background:#484f58}
.msg{padding:10px;border-radius:6px;margin-bottom:12px;font-size:13px;display:none}
.msg.show{display:block}
.msg.success{background:#238636;color:#fff}
.msg.error{background:#da3633;color:#fff}
a.stat{display:block;text-decoration:none;transition:all .2s}
a.stat:hover{border-color:#1f6feb;transform:translateY(-2px)}
@media(max-width:480px){.stat .val{font-size:18px}.nav a{font-size:13px;padding:10px}}
</style></head><body>
)rawliteral";

const char HTML_NAV[] PROGMEM = R"rawliteral(
<div class="nav">
<a href="/">Dashboard</a>
<a href="/settings">Settings</a>
<a href="/wifi">WiFi AP</a>
</div>
)rawliteral";

// ================= DASHBOARD =================
const char HTML_DASH[] PROGMEM = R"rawliteral(
<div id="stateBar" class="state warmup">Connecting...</div>
<div class="card"><h2>Sensor Readings</h2><div class="grid">
<div class="stat" id="s-temp"><div class="lbl">Temperature</div><div class="val" id="v-temp">--</div></div>
<div class="stat" id="s-hum"><div class="lbl">Humidity</div><div class="val" id="v-hum">--</div></div>
<div class="stat" id="s-gas"><div class="lbl">Gas</div><div class="val" id="v-gas">--</div></div>
<div class="stat" id="s-light"><div class="lbl">Light</div><div class="val" id="v-light">--</div></div>
<div class="stat" id="s-motion"><div class="lbl">Motion</div><div class="val" id="v-motion">--</div></div>
<div class="stat" id="s-relay"><div class="lbl">Relay</div><div class="val" id="v-relay">--</div></div>
</div></div>
<div class="card"><h2>System</h2><div class="grid">
<div class="stat"><div class="lbl">Uptime</div><div class="val" id="v-uptime">--</div></div>
<div class="stat"><div class="lbl">Heap Free</div><div class="val" id="v-heap">--</div></div>
<div class="stat"><div class="lbl">Muted</div><div class="val" id="v-mute">--</div></div>
</div></div>
<div class="card"><button class="btn secondary" onclick="location.reload()">Refresh</button></div>
<script>
function fmt(v,d){return v.toFixed(d)}
function upd(){
 fetch('/api/data').then(r=>r.json()).then(d=>{
  var sb=document.getElementById('stateBar');
  sb.className='state '+d.state.toLowerCase();
  sb.textContent=d.stateText;
  document.getElementById('v-temp').textContent=!d.dhtEn?'OFF':(d.dhtErr?'ERR':fmt(d.temp,1)+' C');
  document.getElementById('s-temp').className='stat '+(d.dhtErr?'warn':(d.temp>d.tempThr/10?'alarm':'ok'));
  document.getElementById('v-hum').textContent=!d.dhtEn?'OFF':(d.dhtErr?'ERR':fmt(d.hum,1)+' %');
  document.getElementById('v-gas').textContent=!d.mq2En?'OFF':d.gas;
  document.getElementById('s-gas').className='stat '+(d.gas>d.gasThr?'alarm':'ok');
  document.getElementById('v-light').textContent=!d.ldrEn?'OFF':d.light;
  document.getElementById('v-motion').textContent=!d.pirEn?'OFF':(d.motion?'YES':'NO');
  document.getElementById('s-motion').className='stat '+(d.motion?'warn':'');
  document.getElementById('v-relay').textContent=d.relay?'ON':'OFF';
  document.getElementById('s-relay').className='stat '+(d.relay?'warn':'');
  var sec=Math.floor(d.uptime/1000),m=Math.floor(sec/60),h=Math.floor(m/60);
  document.getElementById('v-uptime').textContent=h+'h '+m%60+'m';
  document.getElementById('v-heap').textContent=Math.floor(d.heap/1024)+'K';
  document.getElementById('v-mute').textContent=d.muted?'YES':'NO';
 }).catch(e=>{});
}
upd();setInterval(upd,2000);
</script>
)rawliteral";

// ================= SETTINGS HUB =================
const char HTML_HUB[] PROGMEM = R"rawliteral(
<div class="card"><h2>Per-Sensor Settings</h2>
<div class="grid">
<a class="stat" href="/settings/dht"><div class="lbl">DHT22</div><div class="val">T/H</div></a>
<a class="stat" href="/settings/mq2"><div class="lbl">MQ2</div><div class="val">Gas</div></a>
<a class="stat" href="/settings/ldr"><div class="lbl">LDR</div><div class="val">Light</div></a>
<a class="stat" href="/settings/pir"><div class="lbl">PIR</div><div class="val">Motion</div></a>
<a class="stat" href="/settings/buzzer"><div class="lbl">Buzzer</div><div class="val">Alarm</div></a>
<a class="stat" href="/settings/relay"><div class="lbl">Relay</div><div class="val">Output</div></a>
<a class="stat" href="/settings/system"><div class="lbl">System</div><div class="val">Timing</div></a>
</div></div>
<div class="card"><h2>Maintenance</h2>
<button class="btn danger" onclick="factoryReset()">Factory Reset</button>
</div>
<script>
function factoryReset(){
 if(!confirm('Factory reset? Device will reboot.'))return;
 fetch('/api/factoryreset',{method:'POST'}).then(()=>{setTimeout(function(){location.reload()},3000)}).catch(e=>{});
}
</script>
)rawliteral";

// ================= DHT PAGE =================
const char HTML_DHT[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>DHT22 — Temperature &amp; Humidity</h2>
<form id="f">
<div class="form-group"><label>Sensor Enabled</label>
<select name="dhtEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Temp Alarm Enabled</label>
<select name="dhtAlarmEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Temp Alarm Threshold (C x10, e.g. 400 = 40.0C)</label>
<input type="number" name="tempThresholdX10" min="-400" max="1250" required></div>
<div class="form-group"><label>Temp Calibration Offset (C x10)</label>
<input type="number" name="dhtCalibTempX10" min="-100" max="100" required></div>
<div class="form-group"><label>Hum Calibration Offset (% x10)</label>
<input type="number" name="dhtCalibHumX10" min="-100" max="100" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save DHT Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/dht').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/dht',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= MQ2 PAGE =================
const char HTML_MQ2[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>MQ2 — Gas Sensor</h2>
<form id="f">
<div class="form-group"><label>Sensor Enabled</label>
<select name="mq2Enabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Gas Alarm Enabled</label>
<select name="mq2AlarmEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Gas Threshold (ADC 0-4095)</label>
<input type="number" name="gasThreshold" min="0" max="4095" required></div>
<div class="form-group"><label>Calibration Offset (raw ADC)</label>
<input type="number" name="gasCalibOffset" min="-1000" max="1000" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save MQ2 Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/mq2').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/mq2',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= LDR PAGE =================
const char HTML_LDR[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>LDR — Light Sensor</h2>
<form id="f">
<div class="form-group"><label>Sensor Enabled</label>
<select name="ldrEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>LDR Dark Threshold (ADC 0-4095)</label>
<input type="number" name="ldrDarkThreshold" min="0" max="4095" required></div>
<div class="form-group"><label>High ADC Means Dark</label>
<select name="ldrHighMeansDark"><option value="1">Yes</option><option value="0">No</option></select></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save LDR Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/ldr').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/ldr',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= PIR PAGE =================
const char HTML_PIR[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>PIR — Motion Sensor</h2>
<form id="f">
<div class="form-group"><label>Sensor Enabled</label>
<select name="pirEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Motion Hold Time (ms)</label>
<input type="number" name="pirHoldTime" min="0" max="3600000" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save PIR Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/pir').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/pir',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= BUZZER PAGE =================
const char HTML_BUZZER[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>Buzzer — Alarm Output</h2>
<form id="f">
<div class="form-group"><label>Buzzer Enabled</label>
<select name="buzzerEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Mute Duration (ms)</label>
<input type="number" name="muteDuration" min="1000" max="3600000" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save Buzzer Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/buzzer').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/buzzer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= RELAY PAGE =================
const char HTML_RELAY[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>Relay — Output Control</h2>
<form id="f">
<div class="form-group"><label>Relay Enabled</label>
<select name="relayEnabled"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="form-group"><label>Relay Hold Time (ms)</label>
<input type="number" name="relayHoldTime" min="0" max="3600000" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save Relay Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/sensor/relay').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/sensor/relay',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= SYSTEM PAGE =================
const char HTML_SYSTEM[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>System Timing</h2>
<form id="f">
<div class="form-group"><label>Warmup Time (ms)</label>
<input type="number" name="warmupTime" min="0" max="600000" required></div>
<div class="form-group"><label>Sensor Read Interval (ms)</label>
<input type="number" name="sensorInterval" min="100" max="60000" required></div>
<div class="form-group"><label>Display Update Interval (ms)</label>
<input type="number" name="displayInterval" min="100" max="10000" required></div>
<div class="form-group"><label>Alarm Latch Time (ms)</label>
<input type="number" name="alarmLatchTime" min="0" max="3600000" required></div>
</form></div>
<div class="card">
<button class="btn" onclick="save()">Save System Settings</button>
<a class="btn secondary" href="/settings">Back to Settings</a>
</div>
<script>
function fill(){fetch('/api/system').then(r=>r.json()).then(c=>{
var f=document.getElementById('f');Object.keys(c).forEach(k=>{if(f.elements[k])f.elements[k].value=c[k]})}).catch(e=>{})}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},3000)}
function save(){
var f=document.getElementById('f'),d={};
for(var i=0;i<f.elements.length;i++){var e=f.elements[i];if(e.name)d[e.name]=parseInt(e.value,10)}
fetch('/api/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
.then(r=>r.json()).then(r=>{showMsg(r.ok?'Saved':'Failed',r.ok)}).catch(e=>showMsg('Network error',false));
}
fill();
</script>
)rawliteral";

// ================= WIFI PAGE =================
const char HTML_WIFI[] PROGMEM = R"rawliteral(
<div id="msg" class="msg"></div>
<div class="card"><h2>WiFi Access Point</h2>
<form id="fw">
<div class="form-group"><label>AP SSID (max 32)</label><input type="text" name="wifiSSID" maxlength="32" required></div>
<div class="form-group"><label>AP Password (8-64)</label><input type="text" name="wifiPass" minlength="8" maxlength="64" required></div>
<div class="form-group"><label>AP Channel (1-13)</label><select name="apChannel">
<option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option>
<option value="5">5</option><option value="6">6</option><option value="7">7</option><option value="8">8</option>
<option value="9">9</option><option value="10">10</option><option value="11">11</option><option value="12">12</option>
<option value="13">13</option></select></div>
</form></div>
<button class="btn" onclick="saveWifi()">Save &amp; Restart AP</button>
<div class="card"><h2>Info</h2>
<div class="grid">
<div class="stat"><div class="lbl">AP IP</div><div class="val" id="v-ip">--</div></div>
<div class="stat"><div class="lbl">Stations</div><div class="val" id="v-sta">--</div></div>
</div></div>
<script>
function fillWifi(){
 fetch('/api/wifi').then(r=>r.json()).then(c=>{
  var f=document.getElementById('fw');
  f.elements['wifiSSID'].value=c.wifiSSID;
  f.elements['wifiPass'].value=c.wifiPass;
  f.elements['apChannel'].value=c.apChannel;
  document.getElementById('v-ip').textContent=c.ip;
  document.getElementById('v-sta').textContent=c.stations;
 }).catch(e=>{});
}
function showMsg(t,ok){var m=document.getElementById('msg');m.textContent=t;m.className='msg show '+(ok?'success':'error');setTimeout(function(){m.className='msg'},4000)}
function saveWifi(){
 var f=document.getElementById('fw');
 var pass=f.elements['wifiPass'].value;
 if(pass==='********'){showMsg('Enter a new password (current one is hidden)',false);return}
 var d={wifiSSID:f.elements['wifiSSID'].value,wifiPass:pass,apChannel:parseInt(f.elements['apChannel'].value,10)};
 if(d.wifiSSID.length<1||d.wifiSSID.length>32){showMsg('SSID must be 1-32 chars',false);return}
 if(d.wifiPass.length<8||d.wifiPass.length>64){showMsg('Password must be 8-64 chars',false);return}
 fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
 .then(r=>r.json()).then(r=>{if(r.ok){showMsg('Saved. Restarting AP...',true);setTimeout(function(){location.reload()},5000)}else{showMsg('Save failed',false)}})
 .catch(e=>showMsg('Network error',false));
}
fillWifi();setInterval(fillWifi,3000);
</script>
)rawliteral";

// ================= PAGE SENDER =================
void sendPageWithBody(const char* body) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(HTML_HEAD);
  server.sendContent_P(HTML_NAV);
  server.sendContent(body);
  server.sendContent(F("</body></html>"));
}

// ================= HANDLERS =================
void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(HTML_HEAD);
  server.sendContent_P(HTML_NAV);
  server.sendContent_P(HTML_DASH);
  server.sendContent(F("</body></html>"));
}

void handleSettingsHub()   { sendPageWithBody(HTML_HUB); }
void handleSettingsDht()   { sendPageWithBody(HTML_DHT); }
void handleSettingsMq2()   { sendPageWithBody(HTML_MQ2); }
void handleSettingsLdr()   { sendPageWithBody(HTML_LDR); }
void handleSettingsPir()   { sendPageWithBody(HTML_PIR); }
void handleSettingsBuzzer(){ sendPageWithBody(HTML_BUZZER); }
void handleSettingsRelay() { sendPageWithBody(HTML_RELAY); }
void handleSettingsSystem(){ sendPageWithBody(HTML_SYSTEM); }
void handleWifiPage()      { sendPageWithBody(HTML_WIFI); }

void handleApiData() {
  const char* stateName = "WARMUP";
  const char* stateText = "Warming Up";
  switch (currentState) {
    case STATE_NORMAL: stateName = "NORMAL"; stateText = "System Normal"; break;
    case STATE_ALARM: stateName = "ALARM"; stateText = buzzerMuted ? "ALARM (MUTED)" : "ALARM ACTIVE"; break;
    case STATE_SENSOR_FAIL: stateName = "FAIL"; stateText = "Sensor Failure"; break;
    default: break;
  }
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"temp\":%.1f,\"hum\":%.1f,\"gas\":%lld,\"light\":%lld,"
    "\"motion\":%d,\"relay\":%d,\"dhtErr\":%d,\"muted\":%d,"
    "\"dhtEn\":%d,\"mq2En\":%d,\"ldrEn\":%d,\"pirEn\":%d,"
    "\"state\":\"%s\",\"stateText\":\"%s\","
    "\"uptime\":%llu,\"heap\":%u,\"gasThr\":%ld,\"tempThr\":%ld}",
    (double)currentData.temperature, (double)currentData.humidity,
    (long long)currentData.gasLevel, (long long)currentData.lightLevel,
    currentData.motionDetected ? 1 : 0, relayState ? 1 : 0,
    currentData.dhtError ? 1 : 0, buzzerMuted ? 1 : 0,
    (int)config.dhtEnabled, (int)config.mq2Enabled, (int)config.ldrEnabled, (int)config.pirEnabled,
    stateName, stateText,
    (unsigned long long)(millis() - bootTime), ESP.getFreeHeap(),
    (long)config.gasThreshold, (long)config.tempThresholdX10);
  server.send(200, "application/json", jsonBuffer);
}

template<typename T>
T clampTo(T v, long lo, long hi) {
  if ((long long)v < (long long)lo) return (T)lo;
  if ((long long)v > (long long)hi) return (T)hi;
  return v;
}

bool parseJsonInt(const String& body, const char* key, int64_t& out) {
  String pat = String("\"") + key + "\":";
  int idx = body.indexOf(pat);
  if (idx < 0) return false;
  int start = idx + pat.length();
  while (start < (int)body.length() &&
         (body[start] == ' ' || body[start] == '\t' || body[start] == '\n' || body[start] == '\r')) start++;
  int end = start;
  if (end < (int)body.length() && body[end] == '-') end++;
  int digitStart = end;
  while (end < (int)body.length() && isdigit(body[end])) end++;
  if (end == digitStart) return false;
  if (end < (int)body.length() && (body[end] == '.' || body[end] == 'e' || body[end] == 'E')) return false;
  out = body.substring(start, end).toInt();
  return true;
}

String jsonExtractString(const String& body, const char* key) {
  String pat = String("\"") + key + "\":";
  int idx = body.indexOf(pat);
  if (idx < 0) return String("");
  int start = idx + pat.length();
  while (start < (int)body.length() &&
         (body[start] == ' ' || body[start] == '\t' || body[start] == '\n' || body[start] == '\r')) start++;
  if (start >= (int)body.length() || body[start] != '"') return String("");
  start++;
  String out;
  for (int i = start; i < (int)body.length(); i++) {
    char c = body[i];
    if (c == '\\' && i + 1 < (int)body.length()) {
      char n = body[i+1];
      if (n == '"' || n == '\\' || n == '/') { out += n; i++; continue; }
      if (n == 'n') { out += '\n'; i++; continue; }
      if (n == 't') { out += '\t'; i++; continue; }
      if (n == 'r') { out += '\r'; i++; continue; }
      out += n; i++; continue;
    }
    if (c == '"') return out;
    out += c;
  }
  return String("");
}

// ---- DHT API ----
void handleApiDhtGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"dhtEnabled\":%d,\"dhtAlarmEnabled\":%d,\"tempThresholdX10\":%ld,"
    "\"dhtCalibTempX10\":%ld,\"dhtCalibHumX10\":%ld}",
    (int)config.dhtEnabled, (int)config.dhtAlarmEnabled,
    (long)config.tempThresholdX10,
    (long)config.dhtCalibTempX10, (long)config.dhtCalibHumX10);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiDhtPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  if (parseJsonInt(body,"dhtEnabled",v))        config.dhtEnabled        = v ? 1 : 0;
  if (parseJsonInt(body,"dhtAlarmEnabled",v))   config.dhtAlarmEnabled   = v ? 1 : 0;
  if (parseJsonInt(body,"tempThresholdX10",v))  config.tempThresholdX10  = clampTo<int32_t>((int32_t)v, -400, 1250);
  if (parseJsonInt(body,"dhtCalibTempX10",v))   config.dhtCalibTempX10   = clampTo<int32_t>((int32_t)v, -100, 100);
  if (parseJsonInt(body,"dhtCalibHumX10",v))    config.dhtCalibHumX10    = clampTo<int32_t>((int32_t)v, -100, 100);
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- MQ2 API ----
void handleApiMq2Get() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"mq2Enabled\":%d,\"mq2AlarmEnabled\":%d,\"gasThreshold\":%ld,\"gasCalibOffset\":%ld}",
    (int)config.mq2Enabled, (int)config.mq2AlarmEnabled,
    (long)config.gasThreshold, (long)config.gasCalibOffset);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiMq2Post() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  if (parseJsonInt(body,"mq2Enabled",v))      config.mq2Enabled      = v ? 1 : 0;
  if (parseJsonInt(body,"mq2AlarmEnabled",v)) config.mq2AlarmEnabled = v ? 1 : 0;
  if (parseJsonInt(body,"gasThreshold",v))    config.gasThreshold    = clampTo<int32_t>((int32_t)v, 0, 4095);
  if (parseJsonInt(body,"gasCalibOffset",v))  config.gasCalibOffset  = clampTo<int32_t>((int32_t)v, -1000, 1000);
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- LDR API ----
void handleApiLdrGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"ldrEnabled\":%d,\"ldrDarkThreshold\":%ld,\"ldrHighMeansDark\":%d}",
    (int)config.ldrEnabled, (long)config.ldrDarkThreshold, (int)config.ldrHighMeansDark);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiLdrPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  if (parseJsonInt(body,"ldrEnabled",v))       config.ldrEnabled       = v ? 1 : 0;
  if (parseJsonInt(body,"ldrDarkThreshold",v)) config.ldrDarkThreshold = clampTo<int32_t>((int32_t)v, 0, 4095);
  if (parseJsonInt(body,"ldrHighMeansDark",v)) config.ldrHighMeansDark = v ? 1 : 0;
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- PIR API ----
void handleApiPirGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"pirEnabled\":%d,\"pirHoldTime\":%lu}",
    (int)config.pirEnabled, (unsigned long)config.pirHoldTime);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiPirPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  uint32_t oldHold = config.pirHoldTime > config.relayHoldTime ? config.pirHoldTime : config.relayHoldTime;
  bool holdChanged = false;
  if (parseJsonInt(body,"pirEnabled",v))  config.pirEnabled  = v ? 1 : 0;
  if (parseJsonInt(body,"pirHoldTime",v)) { config.pirHoldTime = clampTo<uint32_t>((uint32_t)v, 0, 3600000UL); holdChanged = true; }
  uint32_t newHold = config.pirHoldTime > config.relayHoldTime ? config.pirHoldTime : config.relayHoldTime;
  if (holdChanged && newHold > oldHold && ELAPSED(millis(), lastMotionTime) >= oldHold) {
    lastMotionTime = millis() - newHold;
  }
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- Buzzer API ----
void handleApiBuzzerGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"buzzerEnabled\":%d,\"muteDuration\":%lu}",
    (int)config.buzzerEnabled, (unsigned long)config.muteDuration);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiBuzzerPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  if (parseJsonInt(body,"buzzerEnabled",v)) config.buzzerEnabled = v ? 1 : 0;
  if (parseJsonInt(body,"muteDuration",v))  config.muteDuration  = clampTo<uint32_t>((uint32_t)v, 1000, 3600000UL);
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- Relay API ----
void handleApiRelayGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"relayEnabled\":%d,\"relayHoldTime\":%lu}",
    (int)config.relayEnabled, (unsigned long)config.relayHoldTime);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiRelayPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  uint32_t oldHold = config.pirHoldTime > config.relayHoldTime ? config.pirHoldTime : config.relayHoldTime;
  bool holdChanged = false;
  if (parseJsonInt(body,"relayEnabled",v))  config.relayEnabled  = v ? 1 : 0;
  if (parseJsonInt(body,"relayHoldTime",v)) { config.relayHoldTime = clampTo<uint32_t>((uint32_t)v, 0, 3600000UL); holdChanged = true; }
  uint32_t newHold = config.pirHoldTime > config.relayHoldTime ? config.pirHoldTime : config.relayHoldTime;
  if (holdChanged && newHold > oldHold && ELAPSED(millis(), lastMotionTime) >= oldHold) {
    lastMotionTime = millis() - newHold;
  }
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- System API ----
void handleApiSystemGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"warmupTime\":%lu,\"sensorInterval\":%lu,\"displayInterval\":%lu,\"alarmLatchTime\":%lu}",
    (unsigned long)config.warmupTime, (unsigned long)config.sensorInterval,
    (unsigned long)config.displayInterval, (unsigned long)config.alarmLatchTime);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiSystemPost() {
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false}"); return; }
  String body = server.arg("plain");
  int64_t v;
  if (parseJsonInt(body,"warmupTime",v))      config.warmupTime      = clampTo<uint32_t>((uint32_t)v, 0, 600000UL);
  if (parseJsonInt(body,"sensorInterval",v))  config.sensorInterval  = clampTo<uint32_t>((uint32_t)v, 100, 60000UL);
  if (parseJsonInt(body,"displayInterval",v)) config.displayInterval = clampTo<uint32_t>((uint32_t)v, 100, 10000UL);
  if (parseJsonInt(body,"alarmLatchTime",v))  config.alarmLatchTime  = clampTo<uint32_t>((uint32_t)v, 0, 3600000UL);
  if (config.dhtEnabled && config.sensorInterval < 2000) config.sensorInterval = 2000;
  configDirty = true;
  server.send(200,"application/json","{\"ok\":true}");
}

// ---- WiFi API ----
void handleApiWifiGet() {
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"wifiSSID\":\"%s\",\"wifiPass\":\"********\",\"apChannel\":%d,"
    "\"ip\":\"%s\",\"stations\":%d}",
    config.wifiSSID, (int)config.apChannel,
    WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum());
  server.send(200, "application/json", jsonBuffer);
}

void handleApiWifiPost() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  String body = server.arg("plain");
  String ssid = jsonExtractString(body, "wifiSSID");
  String pass = jsonExtractString(body, "wifiPass");
  int64_t ch = 6;
  parseJsonInt(body, "apChannel", ch);

  if (ssid.length() < 1 || ssid.length() > 32 ||
      pass.length() < 8 || pass.length() > 64 ||
      ch < 1 || ch > 13) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  memset(config.wifiSSID, 0, sizeof(config.wifiSSID));
  memset(config.wifiPass, 0, sizeof(config.wifiPass));
  strncpy(config.wifiSSID, ssid.c_str(), sizeof(config.wifiSSID) - 1);
  strncpy(config.wifiPass, pass.c_str(), sizeof(config.wifiPass) - 1);
  config.apChannel = (uint8_t)ch;
  saveConfig();

  server.send(200, "application/json", "{\"ok\":true}");

  restartApPending = true;
  restartApAt = millis() + 500;
}

void handleApiFactoryReset() {
  shuttingDown = true;
  server.send(200, "application/json", "{\"ok\":true}");
  delay(100);
  preferences.begin("iot_cfg", false);
  preferences.clear();
  preferences.end();
  delay(200);
  ESP.restart();
}

void handleNotFound() {
  String uri = server.uri();
  if (server.method() != HTTP_GET || uri.startsWith("/api/")) {
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
    return;
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ================= CAPTIVE PORTAL =================
void startCaptivePortal() {
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.onNotFound(handleNotFound);
}

// ================= WiFi AP =================
void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(config.wifiSSID, config.wifiPass, config.apChannel, 0, 4);
  delay(100);
}

// ================= FACTORY RESET =================
void performFactoryReset() {
  preferences.begin("iot_cfg", false);
  preferences.clear();
  preferences.end();
  delay(200);
  ESP.restart();
}

// ================= SENSOR LOGIC =================
void readSensors() {
  if (config.dhtEnabled) {
    if (ELAPSED(millis(), lastDhtRead) >= 2000) {
      lastDhtRead = millis();
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (isnan(t) || isnan(h)) {
        dhtErrorCount++;
        if (dhtErrorCount > 3) currentData.dhtError = true;
      } else {
        dhtErrorCount = 0;
        currentData.dhtError = false;
        currentData.temperature = t + (float)config.dhtCalibTempX10 / 10.0f;
        currentData.humidity    = h + (float)config.dhtCalibHumX10  / 10.0f;
      }
    }
  } else {
    currentData.dhtError = false;
    currentData.temperature = 0.0f;
    currentData.humidity = 0.0f;
  }

  if (config.ldrEnabled) {
    currentData.lightLevel = (int64_t)analogRead(PIN_LDR);
  } else {
    currentData.lightLevel = 0;
  }

  if (config.mq2Enabled) {
    int64_t raw = (int64_t)analogRead(PIN_MQ2) + (int64_t)config.gasCalibOffset;
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    currentData.gasLevel = raw;
  } else {
    currentData.gasLevel = 0;
  }

  if (config.pirEnabled) {
    currentData.motionDetected = digitalRead(PIN_PIR) ? true : false;
    if (currentData.motionDetected) motionLatch = true;
  } else {
    currentData.motionDetected = false;
  }
}

void evaluateLogic(uint32_t now) {
  bool gasDetected  = config.mq2Enabled && config.mq2AlarmEnabled &&
                      (currentData.gasLevel > (int64_t)config.gasThreshold);
  bool tempDetected = config.dhtEnabled && config.dhtAlarmEnabled &&
                      !currentData.dhtError &&
                      ((currentData.temperature * 10.0f) > (float)config.tempThresholdX10);

  if (gasDetected || tempDetected) {
    alarmLatched = true;
    lastSafeTime = now;
  }

  if (!gasDetected && !tempDetected) {
    if (ELAPSED(now, lastSafeTime) >= (uint32_t)config.alarmLatchTime) {
      alarmLatched = false;
      buzzerMuted = false;
    }
  }

  if (currentData.dhtError && config.dhtEnabled) buzzerMuted = false;

  if (alarmLatched) currentState = STATE_ALARM;
  else if (currentData.dhtError && config.dhtEnabled) currentState = STATE_SENSOR_FAIL;
  else currentState = STATE_NORMAL;

  if (currentState == STATE_ALARM) {
    if (config.buzzerEnabled && !buzzerMuted) BUZZER_ON(); else BUZZER_OFF();
    LED_ON();
  } else if (currentState == STATE_SENSOR_FAIL) {
    if (config.buzzerEnabled && failBlinkState) BUZZER_ON(); else BUZZER_OFF();
    LED_ON();
  } else {
    BUZZER_OFF();
    LED_OFF();
  }

  bool motionNow = currentData.motionDetected || motionLatch;
  motionLatch = false;

  bool isDark = config.ldrHighMeansDark
                  ? (currentData.lightLevel > (int64_t)config.ldrDarkThreshold)
                  : (currentData.lightLevel < (int64_t)config.ldrDarkThreshold);

  if (config.pirEnabled && motionNow && isDark) {
    lastMotionTime = now;
    relayState = true;
  }

  uint32_t holdTime = config.pirHoldTime > config.relayHoldTime ? config.pirHoldTime : config.relayHoldTime;
  if (config.relayEnabled) {
    if ((ELAPSED(now, lastMotionTime) < holdTime) || gasDetected) {
      relayState = true;
    } else {
      relayState = false;
    }
    if (relayState) RELAY_ON(); else RELAY_OFF();
  } else {
    relayState = false;
    RELAY_OFF();
  }
}

// ================= DISPLAY =================
void updateDisplay() {
  uint32_t now = millis();
  if (currentState == STATE_SENSOR_FAIL) {
    if (ELAPSED(now, lastFailBlinkToggle) >= 500) {
      lastFailBlinkToggle = now;
      failBlinkState = !failBlinkState;
      if (config.buzzerEnabled && failBlinkState) BUZZER_ON(); else BUZZER_OFF();
    }
  }

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

  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  if (!config.dhtEnabled) display.print(F("Temp: DISABLED"));
  else if (currentData.dhtError) display.print(F("Temp: SENSOR ERROR"));
  else { display.print(F("Temp: ")); display.print(currentData.temperature, 1); display.print(F(" C")); }

  display.setCursor(0, 25);
  if (!config.dhtEnabled) display.print(F("Hum:  DISABLED"));
  else if (currentData.dhtError) display.print(F("Hum:  SENSOR ERROR"));
  else { display.print(F("Hum:  ")); display.print(currentData.humidity, 1); display.print(F(" %")); }

  display.setCursor(0, 35);
  if (!config.mq2Enabled) display.print(F("Gas:  DISABLED"));
  else { display.print(F("Gas:  ")); display.print((long)currentData.gasLevel); }

  display.setCursor(0, 45);
  if (!config.ldrEnabled) display.print(F("Light: DISABLED"));
  else { display.print(F("Light:")); display.print((long)currentData.lightLevel); }

  display.setCursor(0, 55);
  if (!config.pirEnabled) display.print(F("Motion: DISABLED"));
  else { display.print(F("Motion: ")); display.print(currentData.motionDetected ? "YES" : "NO"); }

  display.display();

  static uint32_t lastI2cProbe = 0;
  if (ELAPSED(millis(), lastI2cProbe) >= 10000) {
    lastI2cProbe = millis();
    Wire.beginTransmission(SCREEN_ADDRESS);
    if (Wire.endTransmission() != 0) {
      recoverI2C();
      display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    }
  }
}

// ================= BUTTON =================
void handleButton(uint32_t now) {
  bool buttonState = digitalRead(PIN_BUTTON);

  if (buttonState == LOW && lastButtonState == HIGH && ELAPSED(now, lastButtonPress) > BUTTON_DEBOUNCE_MS) {
    lastButtonPress = now;
    buttonHoldStart = now;
    if (alarmLatched && currentState == STATE_ALARM && !buzzerMuted) {
      buzzerMuted = true;
      muteStartTime = now;
    }
  }

  if (buttonState == LOW && buttonHoldStart != 0) {
    if (!alarmLatched && currentState != STATE_ALARM &&
        ELAPSED(now, buttonHoldStart) >= FACTORY_RESET_HOLD_MS && !factoryResetPending) {
      factoryResetPending = true;
      performFactoryReset();
    }
  } else {
    buttonHoldStart = 0;
  }

  lastButtonState = buttonState;

  if (buzzerMuted && ELAPSED(now, muteStartTime) > (uint32_t)config.muteDuration) {
    buzzerMuted = false;
  }
}

// ================= SETUP =================
void setup() {
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  if (esp_task_wdt_init(&twdt_config) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&twdt_config);
  }
#else
  esp_task_wdt_init(&twdt_config);
#endif
  esp_task_wdt_add(NULL);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_MQ2, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  BUZZER_OFF();
  RELAY_OFF();
  LED_OFF();

  dht.begin();
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    recoverI2C();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  }

  loadConfig();

  memset(&currentData, 0, sizeof(currentData));

  WiFi.mode(WIFI_AP);
  startAP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettingsHub);
  server.on("/settings/dht", HTTP_GET, handleSettingsDht);
  server.on("/settings/mq2", HTTP_GET, handleSettingsMq2);
  server.on("/settings/ldr", HTTP_GET, handleSettingsLdr);
  server.on("/settings/pir", HTTP_GET, handleSettingsPir);
  server.on("/settings/buzzer", HTTP_GET, handleSettingsBuzzer);
  server.on("/settings/relay", HTTP_GET, handleSettingsRelay);
  server.on("/settings/system", HTTP_GET, handleSettingsSystem);
  server.on("/wifi", HTTP_GET, handleWifiPage);

  server.on("/api/data", HTTP_GET, handleApiData);

  server.on("/api/sensor/dht", HTTP_GET, handleApiDhtGet);
  server.on("/api/sensor/dht", HTTP_POST, handleApiDhtPost);
  server.on("/api/sensor/mq2", HTTP_GET, handleApiMq2Get);
  server.on("/api/sensor/mq2", HTTP_POST, handleApiMq2Post);
  server.on("/api/sensor/ldr", HTTP_GET, handleApiLdrGet);
  server.on("/api/sensor/ldr", HTTP_POST, handleApiLdrPost);
  server.on("/api/sensor/pir", HTTP_GET, handleApiPirGet);
  server.on("/api/sensor/pir", HTTP_POST, handleApiPirPost);
  server.on("/api/sensor/buzzer", HTTP_GET, handleApiBuzzerGet);
  server.on("/api/sensor/buzzer", HTTP_POST, handleApiBuzzerPost);
  server.on("/api/sensor/relay", HTTP_GET, handleApiRelayGet);
  server.on("/api/sensor/relay", HTTP_POST, handleApiRelayPost);

  server.on("/api/system", HTTP_GET, handleApiSystemGet);
  server.on("/api/system", HTTP_POST, handleApiSystemPost);

  server.on("/api/wifi", HTTP_GET, handleApiWifiGet);
  server.on("/api/wifi", HTTP_POST, handleApiWifiPost);

  server.on("/api/factoryreset", HTTP_POST, handleApiFactoryReset);

  server.onNotFound(handleNotFound);
  server.begin();

  startCaptivePortal();

  bootTime = millis();
  lastSafeTime = millis();
  lastMotionTime = millis() - config.relayHoldTime;
  lastSensorRead = 0;
  lastDisplayUpdate = 0;
  lastNvsSave = millis();
}

// ================= LOOP =================
void loop() {
  esp_task_wdt_reset();
  if (shuttingDown) { delay(10); return; }
  uint32_t now = millis();

  server.handleClient();
  dnsServer.processNextRequest();

  handleButton(now);

  if (restartApPending && (int32_t)(millis() - restartApAt) >= 0) {
    restartApPending = false;
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(config.wifiSSID, config.wifiPass, config.apChannel, 0, 4);
    delay(200);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    lastDisplayUpdate = millis();
  }

  if (currentState == STATE_WARMUP && ELAPSED(now, bootTime) >= config.warmupTime) {
    currentState = STATE_NORMAL;
  }

  if (ELAPSED(now, lastSensorRead) >= config.sensorInterval) {
    lastSensorRead = now;
    readSensors();
    if (currentState != STATE_WARMUP) {
      evaluateLogic(now);
    }
  }

  if (ELAPSED(now, lastDisplayUpdate) >= config.displayInterval) {
    lastDisplayUpdate = now;
    if (config.pirEnabled && digitalRead(PIN_PIR)) motionLatch = true;
    updateDisplay();
  }

  if (configDirty && ELAPSED(now, lastNvsSave) >= NVS_SAVE_MIN_INTERVAL_MS) {
    saveConfig();
  }

  delay(1);
}
