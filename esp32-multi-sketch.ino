#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <strings.h>

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error This sketch requires the classic ESP32 target and this exact GPIO map.
#endif

namespace Field {
using Ms = uint64_t;
constexpr char Version[] = "1.0.1";
constexpr char PortalHost[] = "192.168.4.1";
constexpr char PortalUrl[] = "http://192.168.4.1";
constexpr uint8_t DhtPin=4, GasPin=35, LightPin=34, PirPin=27, BootPin=0, Sda=21, Scl=22;
constexpr gpio_num_t OutPins[]={GPIO_NUM_25,GPIO_NUM_15,GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_5,GPIO_NUM_26,GPIO_NUM_33};
constexpr Ms SaveSpacing=15000, SaveWindow=3600000, ResetHold=5000;
constexpr size_t SaveBudget=12;
static_assert(ARDUINO_RUNNING_CORE==1,"Select Core 1 for the Arduino loop task.");
constexpr uint64_t RecordMagic=UINT64_C(0x4649454c444e4f44), Schema=1;
#define SETTINGS(X) \
 X(OledOn,oled_on,0,1,1) X(OledAddr,oled_addr,60,61,60) X(OledContrast,oled_contrast,1,255,127) X(OledRotate,oled_rotate,0,1,0) X(OledPage,oled_page,0,2,0) X(OledMs,oled_ms,500,10000,1000) \
 X(DhtOn,dht_on,0,1,1) X(DhtMs,dht_ms,2500,60000,3000) X(TempMode,temp_mode,0,2,0) X(TempLimit,temp_limit,-400,800,350) X(TempHys,temp_hys,0,300,10) X(HumMode,hum_mode,0,2,0) X(HumLimit,hum_limit,0,1000,750) X(HumHys,hum_hys,0,300,30) \
 X(GasOn,gas_on,0,1,1) X(GasWarm,gas_warm,30,3600,180) X(GasMode,gas_mode,0,2,0) X(GasLimit,gas_limit,0,4095,2000) X(GasHys,gas_hys,0,4095,100) \
 X(LightOn,light_on,0,1,1) X(LightMode,light_mode,0,2,0) X(LightLimit,light_limit,0,4095,700) X(LightHys,light_hys,0,4095,80) \
 X(PirOn,pir_on,0,1,1) X(PirWarm,pir_warm,0,300,30) X(PirAlarm,pir_alarm,0,1,0) X(PirHold,pir_hold,0,600,10) \
 X(BuzzOn,buzz_on,0,1,0) X(BuzzMask,buzz_mask,0,15,15) X(BuzzPeriod,buzz_period,200,10000,1000) X(BuzzPulse,buzz_pulse,50,10000,150) \
 X(LedOn,led_on,0,1,0) X(LedMask,led_mask,0,15,15) X(LedPeriod,led_period,0,10000,0) \
 X(RelayMode,relay_mode,0,3,0) X(DhtRelay,dht_relay,0,3,3) X(GasRelay,gas_relay,0,3,3) X(LightRelay,light_relay,0,3,3) X(PirRelay,pir_relay,0,3,3) \
 X(ApChan,ap_chan,1,11,6) X(ApMax,ap_max,1,4,4)
#define AS_ENUM(id,key,lo,hi,def) id,
enum Key : size_t { SETTINGS(AS_ENUM) KeyCount };
#undef AS_ENUM
struct Spec { const char *name; int64_t lo,hi,initial; };
#define AS_SPEC(id,key,lo,hi,def) {#key,lo,hi,def},
constexpr Spec Specs[]={SETTINGS(AS_SPEC)};
#undef AS_SPEC
#undef SETTINGS
static_assert(KeyCount+3 <= 64,"Form key bitset exhausted");

struct Config {
  int64_t v[KeyCount];
  char ssid[33], pass[64];
  uint8_t reserved[7];
  int64_t operator[](Key k) const { return v[k]; }
};
struct Record { uint64_t magic,schema,seq; Config cfg; uint32_t crc,reserved; };
static_assert(sizeof(Config)==8*KeyCount+104 && sizeof(Record)==sizeof(Config)+32,"Unexpected storage layout");
struct Rule { Key mode,limit,hys; int64_t lo,hi; };
constexpr Rule Temp{TempMode,TempLimit,TempHys,-400,800}, Hum{HumMode,HumLimit,HumHys,0,1000};
constexpr Rule Gas{GasMode,GasLimit,GasHys,0,4095}, Light{LightMode,LightLimit,LightHys,0,4095};
constexpr Rule Rules[]={Temp,Hum,Gas,Light};

enum Reading : uint8_t { Disabled, Warming, Ready, Fault };
enum Store : uint8_t { Starting, Defaults, Healthy, Recovered, StorageError };
enum Result : uint8_t { NoJob, Pending, Saved, Failed };
struct View {
  Reading state[4];
  int64_t temp,hum,gas,light;
  bool pir,held,inhibit;
  uint8_t alarms,outputs;
  Ms warmGas,warmPir;
};
struct Climate { int64_t temp,hum; Ms at; bool valid; uint64_t failures; };
struct Job { Config cfg; uint64_t id; bool factory,wifi; };
struct Meta {
  uint64_t rev,job;
  Ms deadline,lastAttempt,writeTimes[SaveBudget];
  size_t writeIndex,writeCount;
  Store store;
  Result result;
  bool ready,haveAttempt,resetRequested,resetPending,oledOK,workerFault;
};

portMUX_TYPE guard=portMUX_INITIALIZER_UNLOCKED;
Config live{};
View sensors{};
Climate climate{};
Job queued{};
Meta meta{};
char csrf[33]{}, apSsid[33]{};

// ========================================================
// Shared state, settings validation, and rule evaluation
// ========================================================
Ms nowMs() { return static_cast<Ms>(esp_timer_get_time())/1000; }

void yieldMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1); }

Config defaults() {
  Config c{};
  for (size_t i=0;i<KeyCount;++i) c.v[i]=Specs[i].initial;
  strcpy(c.ssid,"ESP32_Multi_Sensors");
  strcpy(c.pass,"12345678");
  return c;
}

bool printable(const char *s,size_t cap,size_t min) {
  const size_t n=strnlen(s,cap);
  if (n<min || n==cap) return false;
  for (size_t i=0;i<n;++i) if (s[i]<32 || s[i]>126) return false;
  return true;
}

bool validConfig(const Config &c) {
  if (!printable(c.ssid,sizeof c.ssid,1) || !printable(c.pass,sizeof c.pass,8)) return false;
  bool ssidVisible=false;
  for (const char *s=c.ssid;*s;++s) if (*s!=' ') ssidVisible=true;
  if (!ssidVisible) return false;
  for (size_t i=0;i<KeyCount;++i) if (c.v[i]<Specs[i].lo || c.v[i]>Specs[i].hi) return false;
  for (const auto &r:Rules) {
    if (c[r.mode]==1 && c[r.limit]-c[r.hys]<r.lo) return false;
    if (c[r.mode]==2 && c[r.limit]+c[r.hys]>r.hi) return false;
  }
  for (const uint8_t b:c.reserved) if (b) return false;
  return c[BuzzPulse]<=c[BuzzPeriod] && (!c[LedPeriod] || c[LedPeriod]>=200);
}

Ms saveAt(const Meta &m) {
  Ms next=m.haveAttempt ? m.lastAttempt+SaveSpacing : 0;
  if (m.writeCount==SaveBudget) {
    const Ms budget=m.writeTimes[m.writeIndex]+SaveWindow;
    if (budget>next) next=budget;
  }
  return next;
}

bool sameWifi(const Config &a,const Config &b) {
  return strcmp(a.ssid,b.ssid)==0 && strcmp(a.pass,b.pass)==0 && a[ApChan]==b[ApChan] && a[ApMax]==b[ApMax];
}

bool ruleActive(const Config &c,const Rule &r,int64_t val,bool prev) {
  const auto mode=c[r.mode], limit=c[r.limit], h=c[r.hys];
  if (mode==1) return prev && h ? val>limit-h : val>=limit;
  if (mode==2) return prev && h ? val<limit+h : val<=limit;
  return false;
}

bool sameRule(const Config &a,const Config &b,const Rule &r) {
  return a[r.mode]==b[r.mode] && a[r.limit]==b[r.limit] && a[r.hys]==b[r.hys];
}

uint8_t relayBits(const Config &c,uint8_t alarms) {
  constexpr Key route[]={DhtRelay,GasRelay,LightRelay,PirRelay};
  uint8_t bits=0;
  for (size_t i=0;i<4;++i) if (alarms&(1u<<i)) {
    const auto mode=c[RelayMode]&c[route[i]];
    if (mode&1) bits|=1;
    if (mode&2) bits|=1u<<(i+1);
  }
  return bits;
}

void snapshot(Config &c,Meta &m) {
  portENTER_CRITICAL(&guard);
  c=live; m=meta;
  portEXIT_CRITICAL(&guard);
}

uint32_t crc32(const void *mem,size_t n) {
  auto p=static_cast<const uint8_t*>(mem);
  uint32_t crc=UINT32_MAX;
  while(n--) {
    crc^=*p++;
    for (unsigned i=0;i<8;++i) crc=(crc>>1)^(0xedb88320u & (0u-(crc&1u)));
  }
  return ~crc;
}

bool validRecord(const Record &r) {
  return r.magic==RecordMagic && r.schema==Schema && r.seq && !r.reserved &&
         r.crc==crc32(&r,offsetof(Record,crc)) && validConfig(r.cfg);
}

// ========================================================
// Transactional Preferences storage and factory reset queue
// ========================================================
constexpr const char *Bank[]={"slot0","slot1"};

bool readBank(Preferences &p,int bank,Record &r) {
  return p.getBytesLength(Bank[bank])==sizeof r && p.getBytes(Bank[bank],&r,sizeof r)==sizeof r && validRecord(r);
}

bool writeBank(Preferences &p,int bank,uint64_t rev,const Config &c) {
  Record r{RecordMagic,Schema,rev,c,0,0}, check{};
  r.crc=crc32(&r,offsetof(Record,crc));
  return p.putBytes(Bank[bank],&r,sizeof r)==sizeof r && readBank(p,bank,check) && memcmp(&r,&check,sizeof r)==0;
}

struct StorageState { int bank=-1; uint64_t rev=0; };

Store loadStored(Preferences &p,StorageState &state,Config &initial) {
  state=StorageState{};
  Record a{},b{};
  const bool va=readBank(p,0,a),vb=readBank(p,1,b);
  const bool ka=p.isKey(Bank[0]),kb=p.isKey(Bank[1]);
  if (!va && !vb) return ka || kb ? StorageError : Defaults;
  state.bank=va && (!vb || a.seq>=b.seq) ? 0 : 1;
  const Record &r=state.bank==0?a:b;
  state.rev=r.seq; initial=r.cfg;
  return (!va && ka) || (!vb && kb) || (va && vb && a.seq==b.seq && memcmp(&a,&b,sizeof a)!=0) ? Recovered : Healthy;
}

void finishJob(Preferences &p,bool opened,StorageState &state,const Job &j) {
  const int next=state.bank==0?1:0;
  const bool ok=opened && state.rev!=UINT64_MAX && writeBank(p,next,state.rev+1,j.cfg);
  bool mirrored=true;
  if (ok) {
    state.bank=next; ++state.rev;
    if (j.factory) {
      mirrored=state.rev!=UINT64_MAX && writeBank(p,1-state.bank,state.rev+1,j.cfg);
      if (mirrored) { state.bank=1-state.bank; ++state.rev; }
    }
  }
  const Ms finished=nowMs();
  portENTER_CRITICAL(&guard);
  if (ok) {
    live=j.cfg; meta.rev=state.rev; meta.store=mirrored?Healthy:Recovered;
    if (j.factory || j.wifi) meta.deadline=finished+(j.factory?1500:10000);
  } else meta.store=StorageError;
  meta.result=ok?Saved:Failed;
  if (j.factory && !ok) meta.resetPending=false;
  meta.lastAttempt=finished;
  queued=Job{};
  portEXIT_CRITICAL(&guard);
}

void storageTask(void *) {
  Preferences p;
  bool opened=p.begin("fieldnode",false);
  StorageState state;
  Config initial=defaults();
  const Store st=opened?loadStored(p,state,initial):StorageError;
  portENTER_CRITICAL(&guard);
  live=initial; meta.rev=state.rev; meta.store=st; meta.ready=true;
  portEXIT_CRITICAL(&guard);
  for (;;) {
    Job j{};
    portENTER_CRITICAL(&guard);
    if (meta.result==Pending) j=queued;
    portEXIT_CRITICAL(&guard);
    if (j.id) {
      if (!opened) {
        opened=p.begin("fieldnode",false);
        if (opened) {
          Config previous=defaults();
          loadStored(p,state,previous);
        }
      }
      finishJob(p,opened,state,j);
    }
    yieldMs(25);
  }
}

// ========================================================
// DHT22, analog filter, PIR, and fail-off output controller
// ========================================================
void climateTask(void *) {
  DHT dht(DhtPin,DHT22);
  dht.begin();
  Ms last=0;
  uint64_t failures=0;
  for (;;) {
    Config c; Meta m; snapshot(c,m);
    const Ms now=nowMs();
    if (m.ready && c[DhtOn] && now>=last+static_cast<Ms>(c[DhtMs])) {
      last=now;
      const float t=dht.readTemperature(false,true), h=dht.readHumidity();
      const bool ok=isfinite(t) && isfinite(h) && t>=-40 && t<=80 && h>=0 && h<=100;
      if (!ok && failures!=UINT64_MAX) ++failures;
      Climate sample{ok?llroundf(t*10):0,ok?llroundf(h*10):0,last,ok,failures};
      portENTER_CRITICAL(&guard); climate=sample; portEXIT_CRITICAL(&guard);
    }
    yieldMs(25);
  }
}

struct Median {
  uint16_t values[5]{};
  size_t pos=0,count=0;
  void clear() { pos=count=0; }
  void push(uint16_t value) { values[pos]=value; pos=(pos+1)%5; if (count<5) ++count; }
  int64_t value() const {
    uint16_t sorted[5]; memcpy(sorted,values,sizeof sorted);
    for (size_t i=1;i<5;++i) for (size_t j=i;j>0 && sorted[j]<sorted[j-1];--j) {
      const uint16_t n=sorted[j]; sorted[j]=sorted[j-1]; sorted[j-1]=n;
    }
    return sorted[2];
  }
};

struct Debounce {
  bool raw=false,stable=false,qualified=false;
  Ms changed=0;
  bool update(bool current,Ms now,Ms window) {
    if (current!=raw) { raw=current; changed=now; }
    if (now-changed>=window) { qualified=true; stable=raw; }
    return stable;
  }
};

struct AnalogChannel {
  Median filter;
  bool fault=false;
  void clear() { filter.clear(); fault=false; }
  void sample(int raw) {
    fault=raw<0 || raw>4095;
    if (fault) filter.clear();
    else filter.push(static_cast<uint16_t>(raw));
  }
  Reading state(bool enabled,Ms warm=0) const {
    return !enabled?Disabled:fault?Fault:warm || filter.count<5?Warming:Ready;
  }
};

adc_oneshot_unit_handle_t adc=nullptr;
adc_channel_t adcChannels[2]{};
bool adcReady[2]{};

void initAdc() {
  adc_oneshot_unit_init_cfg_t unit{};
  unit.unit_id=ADC_UNIT_1;
  if (adc_oneshot_new_unit(&unit,&adc)!=ESP_OK) return;
  adc_oneshot_chan_cfg_t channel{};
  channel.atten=ADC_ATTEN_DB_12; channel.bitwidth=ADC_BITWIDTH_12;
  const uint8_t pins[]={GasPin,LightPin};
  for (size_t i=0;i<2;++i) {
    adc_unit_t discovered;
    adcReady[i]=adc_oneshot_io_to_channel(pins[i],&discovered,&adcChannels[i])==ESP_OK &&
                discovered==ADC_UNIT_1 && adc_oneshot_config_channel(adc,adcChannels[i],&channel)==ESP_OK;
  }
}

int readAdc(size_t index) {
  int raw=-1;
  return adcReady[index] && adc_oneshot_read(adc,adcChannels[index],&raw)==ESP_OK ? raw : -1;
}

void drive(uint8_t bits) {
  static uint8_t last=0;
  for (size_t i=0;i<sizeof OutPins/sizeof OutPins[0];++i)
    if ((last^bits)&(1u<<i)) gpio_set_level(OutPins[i],(bits>>i)&1u);
  last=bits;
}

void control() {
  static Config previous{};
  static AnalogChannel mq,ldr;
  static Debounce boot,pir;
  static bool armed=false,resetSent=false,buzzPrev=false,ledPrev=false,pirWasActive=false;
  static bool latches[4]{};
  static Ms dhtSince=0,gasSince=0,pirSince=0,pirEnd=0,adcDue=0,buzzStart=0,ledStart=0;
  const Ms now=nowMs();
  Config c; Meta m; Climate d;
  portENTER_CRITICAL(&guard); c=live; m=meta; d=climate; portEXIT_CRITICAL(&guard);
  const bool key=boot.update(digitalRead(BootPin)==LOW,now,40);
  if (boot.qualified && !key && !boot.raw) { armed=true; resetSent=false; }
  if (armed && key && boot.raw && !resetSent && now-boot.changed>=ResetHold) {
    portENTER_CRITICAL(&guard);
    if (!meta.resetPending) meta.resetRequested=true;
    m.resetRequested=meta.resetRequested;
    portEXIT_CRITICAL(&guard);
    resetSent=true;
  }
  if (m.resetRequested && m.ready && !m.workerFault) {
    const Config baseline=defaults();
    portENTER_CRITICAL(&guard);
    if (meta.result!=Pending && !meta.resetPending) {
      if (meta.job!=UINT64_MAX) {
        queued=Job{baseline,++meta.job,true,true};
        meta.result=Pending; meta.resetPending=true; meta.haveAttempt=true;
      } else {
        meta.result=Failed; meta.store=StorageError;
      }
      meta.resetRequested=false;
    }
    m=meta;
    portEXIT_CRITICAL(&guard);
  }
  if (!m.ready || m.workerFault || m.resetPending || m.resetRequested || (m.deadline && now>=m.deadline)) {
    View off{}; off.inhibit=true;
    drive(0);
    for (auto &latch:latches) latch=false;
    buzzPrev=ledPrev=pirWasActive=false; pirEnd=0;
    portENTER_CRITICAL(&guard); sensors=off; portEXIT_CRITICAL(&guard);
    if (m.result!=Pending && !m.resetRequested && m.deadline && now>=m.deadline && boot.qualified && !key && !boot.raw) ESP.restart();
    return;
  }
  View v{};
  if (c[DhtOn] && !previous[DhtOn]) dhtSince=now;
  if (c[GasOn] && !previous[GasOn]) { mq.clear(); gasSince=now; }
  if (c[LightOn] && !previous[LightOn]) ldr.clear();
  if (c[PirOn] && !previous[PirOn]) { pir=Debounce{}; pir.changed=now; pirSince=now; pirEnd=0; pirWasActive=false; }
  if (now>=adcDue) {
    adcDue=now+40;
    if (c[GasOn]) mq.sample(readAdc(0));
    if (c[LightOn]) ldr.sample(readAdc(1));
  }
  const Ms gasEnd=gasSince+1000*static_cast<Ms>(c[GasWarm]);
  const Ms pirReady=pirSince+1000*static_cast<Ms>(c[PirWarm]);
  v.warmGas=c[GasOn] && now<gasEnd ? (gasEnd-now+999)/1000 : 0;
  v.warmPir=c[PirOn] && now<pirReady ? (pirReady-now+999)/1000 : 0;
  const Ms fresh=static_cast<Ms>(c[DhtMs])*2+500;
  v.state[0]=!c[DhtOn]?Disabled:d.at && d.at>=dhtSince && now>=d.at && now-d.at<=fresh ? (d.valid?Ready:Fault) : (now-dhtSince<fresh?Warming:Fault);
  v.state[1]=mq.state(c[GasOn],v.warmGas);
  v.state[2]=ldr.state(c[LightOn]);
  if (c[PirOn]) v.pir=pir.update(digitalRead(PirPin)==HIGH,now,35);
  v.state[3]=!c[PirOn]?Disabled:(v.warmPir || !pir.qualified)?Warming:Ready;
  v.temp=d.temp; v.hum=d.hum; v.gas=mq.filter.value(); v.light=ldr.filter.value();
  const int64_t val[]={v.temp,v.hum,v.gas,v.light};
  constexpr uint8_t which[]={0,0,1,2};
  for (size_t i=0;i<4;++i) {
    latches[i]=v.state[which[i]]==Ready && ruleActive(c,Rules[i],val[i],sameRule(c,previous,Rules[i]) && latches[i]);
    if (latches[i]) v.alarms|=1u<<which[i];
  }
  const bool pirAlarmEnabled=v.state[3]==Ready && c[PirAlarm];
  const bool pirActive=pirAlarmEnabled && v.pir;
  if (!pirAlarmEnabled || c[PirHold]!=previous[PirHold] || pirActive) pirEnd=0;
  else if (pirWasActive) pirEnd=now+static_cast<Ms>(c[PirHold])*1000;
  pirWasActive=pirActive;
  if (pirAlarmEnabled && (pirActive || now<pirEnd)) { v.alarms|=8; v.held=!pirActive; }
  uint8_t bits=relayBits(c,v.alarms);
  const bool buzz=c[BuzzOn] && (c[BuzzMask]&v.alarms), led=c[LedOn] && (c[LedMask]&v.alarms);
  if (buzz && (!buzzPrev || c[BuzzPeriod]!=previous[BuzzPeriod] || c[BuzzPulse]!=previous[BuzzPulse])) buzzStart=now;
  if (led && (!ledPrev || c[LedPeriod]!=previous[LedPeriod])) ledStart=now;
  if (buzz && (now-buzzStart)%c[BuzzPeriod]<static_cast<Ms>(c[BuzzPulse])) bits|=32;
  if (led && (!c[LedPeriod] || (now-ledStart)%c[LedPeriod]<static_cast<Ms>(c[LedPeriod]/2))) bits|=64;
  buzzPrev=buzz; ledPrev=led; previous=c;
  v.outputs=bits;
  drive(bits);
  portENTER_CRITICAL(&guard); sensors=v; portEXIT_CRITICAL(&guard);
}

// ========================================================
// SSD1306 display worker
// ========================================================
uint8_t oledProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

void oledTask(void *) {
  Adafruit_SSD1306 oled(128,64,&Wire,-1,400000,400000);
  bool bus=false,initialized=false,visible=false,wasEnabled=false,lastPage=false;
  uint8_t addr=0,lastWanted=0;
  int64_t contrast=-1,rotation=-1,refresh=-1;
  Ms due=0;
  uint8_t misses=0;
  for (;;) {
    Config c; Meta m; View v;
    portENTER_CRITICAL(&guard); c=live; m=meta; v=sensors; portEXIT_CRITICAL(&guard);
    const Ms now=nowMs();
    if (m.ready && !m.workerFault) {
      const uint8_t wanted=static_cast<uint8_t>(c[OledAddr]);
      const bool enabled=c[OledOn];
      const bool status=c[OledPage]==2 || (c[OledPage]==0 && (now/6000)%2);
      if (enabled!=wasEnabled || wanted!=lastWanted ||
          (initialized && (status!=lastPage || contrast!=c[OledContrast] || rotation!=c[OledRotate] || refresh!=c[OledMs]))) due=0;
      wasEnabled=enabled; lastWanted=wanted;
      if (!enabled || wanted!=addr) {
        if (visible && oledProbe(addr)==0) oled.ssd1306_command(SSD1306_DISPLAYOFF);
        visible=false; initialized=false; contrast=-1;
      }
      if (enabled && now>=due) {
        if (!bus) { bus=Wire.begin(Sda,Scl,400000); Wire.setTimeOut(20); }
        uint8_t probe=bus?oledProbe(wanted):4;
        bool ok=probe==0;
        if (ok && !initialized) {
          ok=oled.begin(SSD1306_SWITCHCAPVCC,wanted,false,false);
          initialized=ok; addr=wanted;
        }
        if (ok) {
          if (contrast!=c[OledContrast]) {
            oled.ssd1306_command(SSD1306_SETCONTRAST);
            oled.ssd1306_command(static_cast<uint8_t>(c[OledContrast]));
            contrast=c[OledContrast];
          }
          rotation=c[OledRotate]; refresh=c[OledMs]; lastPage=status;
          oled.setRotation(rotation?2:0);
          oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
          oled.setCursor(0,0); oled.print(F("FIELD NODE")); oled.drawFastHLine(0,10,128,SSD1306_WHITE);
          char lines[4][22]{};
          if (status) {
            snprintf(lines[0],sizeof lines[0],"AP %s",PortalHost);
            snprintf(lines[1],sizeof lines[1],"Up %" PRIu64 "d %uh",now/86400000,static_cast<unsigned>((now/3600000)%24));
            snprintf(lines[2],sizeof lines[2],"Alarm mask: 0x%02X",v.alarms);
            snprintf(lines[3],sizeof lines[3],"%s",v.inhibit?"Outputs inhibited":m.deadline?"Restart pending":m.store==StorageError?"Storage error":"Local operation");
          } else {
            if (v.state[0]==Ready) snprintf(lines[0],sizeof lines[0],"T %4.1fC RH %4.1f%%",v.temp/10.0,v.hum/10.0);
            else strcpy(lines[0],"DHT22: --");
            if (v.state[1]==Ready) snprintf(lines[1],sizeof lines[1],"MQ2: %" PRId64 " ADC",v.gas); else strcpy(lines[1],"MQ2: --");
            if (v.state[2]==Ready) snprintf(lines[2],sizeof lines[2],"LDR: %" PRId64 " ADC",v.light); else strcpy(lines[2],"LDR: --");
            snprintf(lines[3],sizeof lines[3],"PIR: %s",v.state[3]!=Ready?"--":v.pir?"MOTION":v.held?"HOLD":"CLEAR");
          }
          for (size_t i=0;i<4;++i) { oled.setCursor(0,16+12*i); oled.print(lines[i]); }
          oled.display();
          probe=oledProbe(wanted);
          ok=probe==0;
        }
        if (!ok) {
          initialized=false; contrast=-1; visible=false;
          if (probe==2 || !bus) misses=0;
          else if (probe!=0 && ++misses>=3) { Wire.end(); bus=false; misses=0; }
        } else { visible=true; misses=0; }
        due=now+(ok?static_cast<Ms>(c[OledMs]):5000);
        portENTER_CRITICAL(&guard); meta.oledOK=ok; portEXIT_CRITICAL(&guard);
      }
      if (!enabled) { portENTER_CRITICAL(&guard); meta.oledOK=false; portEXIT_CRITICAL(&guard); }
    }
    yieldMs(50);
  }
}

// ========================================================
// Embedded web console
// ========================================================
constexpr char Page[] PROGMEM = R"FIELDUI(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0d151d">
<title>Field Node · Environmental Monitor</title>
<style>
:root{color-scheme:dark;--bg:#0d151d;--side:#101a23;--panel:#15212b;--panel2:#192734;--line:#2a3947;--text:#f0f5f6;--muted:#a5b5c2;--quiet:#90a4b4;--green:#b8f46b;--amber:#ffc582;--red:#ff9d91;--radius:15px;--mono:ui-monospace,SFMono-Regular,Consolas,monospace;--sidew:248px}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",sans-serif;font-size:14px;line-height:1.5}button,input,select{font:inherit}button,a,input,select{ -webkit-tap-highlight-color:transparent}a{color:inherit}button{cursor:pointer}button:disabled{opacity:.56;cursor:not-allowed}:focus-visible{outline:2px solid var(--green);outline-offset:3px}[hidden]{display:none!important}.icon{width:19px;height:19px;display:inline-block;vertical-align:middle;flex:none;fill:none;stroke:currentColor;stroke-width:1.7;stroke-linecap:round;stroke-linejoin:round}.sr{position:absolute;width:1px;height:1px;overflow:hidden;clip-path:inset(50%);white-space:nowrap}.skip{position:absolute;left:16px;top:-100px;z-index:10;background:var(--green);color:#17220e;padding:10px}.skip:focus{top:10px}.sidebar{position:fixed;z-index:5;inset:0 auto 0 0;width:var(--sidew);background:var(--side);border-right:1px solid var(--line);display:flex;flex-direction:column;overflow:auto}.brand{height:77px;min-height:77px;display:flex;gap:12px;align-items:center;padding:0 21px;text-decoration:none;border-bottom:1px solid var(--line)}.brandmark{width:36px;height:36px;border-radius:10px;background:var(--green);color:#152516;display:grid;place-items:center;flex:none}.brandmark .icon{width:22px;height:22px;stroke-width:1.9}.brand strong{display:block;font-size:16px;font-weight:750;line-height:1.2;letter-spacing:-.35px}.eyebrow,.side-label,.micro,.pin,.chip,.key{font-family:var(--mono);font-size:10px;letter-spacing:.095em;font-weight:650;text-transform:uppercase}.brand .micro{color:var(--quiet);font-size:9px;letter-spacing:.075em}.sidebar nav{padding:16px 13px 20px}.side-label{margin:11px 10px 8px;color:var(--quiet)}.side-label:not(:first-child){margin-top:22px}.navlink{display:flex;gap:10px;align-items:center;min-height:38px;text-decoration:none;color:var(--muted);padding:7px 10px;border:1px solid transparent;border-radius:8px;white-space:nowrap}.navlink .icon{width:16px;height:16px}.navlink:hover{background:#1b2b36;color:var(--text)}.navlink[aria-current="page"]{background:#1f3029;border-color:#3d5434;color:var(--green);font-weight:700}.navlink small{color:var(--quiet);margin-left:auto;font-family:var(--mono);font-size:9px}.side-footer{padding:14px 17px 17px;margin-top:auto}.side-device{border:1px solid var(--line);background:#13202b;border-radius:10px;padding:11px 9px;display:flex;align-items:center;gap:9px}.side-device .icon{color:var(--green)}.side-device strong{display:block;font-size:11px;line-height:1.4}.side-device span{color:var(--muted);font-size:10px}.shell{margin-left:var(--sidew)}.topbar{height:69px;border-bottom:1px solid var(--line);background:#101a23;display:flex;align-items:center;justify-content:space-between;padding:0 31px;gap:12px}.crumb,.top-right{display:flex;align-items:center;gap:9px;min-width:0}.crumb{color:var(--muted);font-size:12px}.crumb .icon{width:13px}.crumb strong{color:var(--text);font-weight:650;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.top-right{flex:none}.top-ip{font:11px var(--mono);color:var(--muted);padding-right:8px}.pill,.tag{display:inline-flex;gap:6px;align-items:center;white-space:nowrap;font-size:11px;font-weight:680;padding:4px 8px;border:1px solid var(--line);border-radius:6px;background:#172530;color:var(--muted)}.dot{width:6px;height:6px;display:inline-block;border-radius:50%;background:currentColor;flex:none}.pill.good,.tag.good{color:var(--green);background:#223127;border-color:#3d5434}.pill.warn,.tag.warn{color:var(--amber);background:#342c23;border-color:#5b4830}.pill.bad,.tag.bad{color:var(--red);background:#392726;border-color:#61403b}.main{max-width:1530px;padding:27px 31px 38px;margin:auto}.banner{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 13px;margin-bottom:14px;border:1px solid #655039;background:#332a22;border-radius:9px;color:#ffe4c6}.banner a{font-weight:700;white-space:nowrap}.hero{position:relative;isolation:isolate;overflow:hidden;min-height:225px;padding:26px 28px;border-radius:var(--radius);border:1px solid #3d5547;background:linear-gradient(95deg,#1a2c28 0%,#162824 50%,#172a27 100%);display:flex;align-items:center;justify-content:space-between;gap:22px}.hero:before{content:"";position:absolute;z-index:-1;inset:0;background-image:linear-gradient(#86af8a12 1px,transparent 1px),linear-gradient(90deg,#86af8a12 1px,transparent 1px);background-size:24px 24px;mask-image:linear-gradient(90deg,transparent,#000 38%)}.hero-art{position:absolute;z-index:-1;right:17%;top:50%;transform:translateY(-50%);width:39%;max-width:380px;min-width:210px;color:#91c981;opacity:.53}.hero-copy{min-width:0}.eyebrow{color:var(--green);display:flex;align-items:center;gap:8px}.eyebrow .dot{box-shadow:0 0 12px #b8f46875}.hero h1,.pagehead h1{font-size:clamp(32px,4vw,51px);line-height:1.06;letter-spacing:-.055em;margin:12px 0 9px;font-weight:760}.hero-copy>p,.pagehead>p{color:#b8c8c9;margin:0;max-width:48ch}.hero-foot{margin-top:19px;display:flex;gap:13px;flex-wrap:wrap;align-items:center;color:#c9d8d1}.hero-foot .micro{font-size:9px}.hero-foot .icon{width:14px;height:14px;color:var(--green);margin-right:5px}.hero-stats{z-index:1;width:190px;flex:none;background:#172923ed;border:1px solid #496453;border-radius:11px;padding:9px 13px}.hstat{display:flex;justify-content:space-between;gap:10px;align-items:center;min-height:37px;color:#c1d0c9;font-size:11px}.hstat+.hstat{border-top:1px solid #3b5345}.hstat b{font:650 11px var(--mono);color:#eaf3eb;white-space:nowrap}.hstat .good{color:var(--green)}.sec-head{display:flex;gap:10px;align-items:flex-end;justify-content:space-between;margin:26px 0 11px}.sec-head h2,.panel-head h2{font-size:20px;line-height:1.2;letter-spacing:-.035em;margin:0}.sec-head p,.panel-head p{font-size:12px;color:var(--muted);margin:4px 0 0}.sec-head .micro{color:var(--quiet);text-align:right}.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}.sensor,.panel{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);min-width:0}.sensor{padding:14px;--accent:#a6c7ff;--accentbg:#25344b}.sensor[data-s="1"]{--accent:#ffd39a;--accentbg:#3e3428}.sensor[data-s="2"]{--accent:#f6e49e;--accentbg:#393625}.sensor[data-s="3"]{--accent:#d5baff;--accentbg:#38304b}.sensor-top{display:flex;align-items:center;justify-content:space-between;gap:6px}.sensor-icon{color:var(--accent);background:var(--accentbg);border-radius:8px;width:33px;height:33px;display:grid;place-items:center}.pin{font-size:9px;color:var(--quiet);border:1px solid var(--line);border-radius:5px;padding:3px 5px;white-space:nowrap}.sensor h3{font-size:13px;color:#dce6ed;margin:11px 0 5px;font-weight:660}.reading{font-variant-numeric:tabular-nums;font-size:clamp(24px,2.6vw,34px);font-weight:730;letter-spacing:-.045em;line-height:1.1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.reading .unit{font-size:13px;font-weight:550;letter-spacing:0;color:var(--muted);margin-left:4px}.subread{font-size:11px;color:var(--muted);margin:6px 0 10px;min-height:17px;overflow-wrap:anywhere}.meter{height:4px;background:#293a49;border-radius:5px;margin:10px 0;overflow:hidden}.meter>span{display:block;width:0;height:100%;border-radius:5px;background:var(--accent);transition:width .25s ease}.sensor-foot{border-top:1px solid var(--line);display:flex;justify-content:space-between;align-items:center;gap:4px;padding-top:9px}.sensor-foot a,.textlink{font-size:11px;color:var(--muted);text-decoration:none;font-weight:650}.sensor-foot a:hover,.textlink:hover{text-decoration:underline;color:var(--text)}.sensor-foot .tag{font-size:9px;padding:2px 5px}.twocol{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(245px,1fr);gap:13px}.panel-head,.panel-body{padding:15px 17px}.panel-head{border-bottom:1px solid var(--line);display:flex;align-items:center;justify-content:space-between;gap:12px}.panel-head h2{font-size:16px}.panel-head .accent{color:var(--green)}.rows{list-style:none;padding:0;margin:0}.rows li{display:flex;align-items:center;justify-content:space-between;gap:9px;padding:8px 0}.rows li+li{border-top:1px solid var(--line)}.rows li>span{color:var(--muted);min-width:0}.rows li>b{font:650 11px var(--mono);text-align:right;overflow-wrap:anywhere}.relay-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px}.relay-cell{background:#111e28;border-radius:8px;padding:7px;border:1px solid var(--line);font-size:10px}.relay-cell:first-child{grid-column:1/-1;display:flex;align-items:center;justify-content:space-between}.relay-cell b{display:block;font-size:11px;font-weight:650;color:#d6e0e8}.relay-cell small{color:var(--quiet);display:block;font:10px var(--mono)}.relay-cell .tag{margin-top:6px;font-size:9px;padding:2px 5px}.relay-cell:first-child .tag{margin:0}.page-footer{margin-top:23px;padding-top:11px;border-top:1px solid var(--line);display:flex;justify-content:space-between;gap:10px;color:var(--quiet);font-size:11px}.menu,.shade{display:none}.pagehead{padding:12px 0 24px}.pagehead h1{font-size:clamp(32px,4vw,46px)}.settings-layout{display:grid;grid-template-columns:minmax(0,1fr) minmax(235px,300px);gap:14px;align-items:start}.context{display:grid;gap:13px}.form-panel{overflow:hidden}.group{padding:16px 18px;border-bottom:1px solid var(--line)}.group h3{margin:0 0 3px;font-size:15px}.group p{color:var(--muted);margin:0 0 12px;font-size:12px}.fields{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:13px;margin-top:10px}.field{min-width:0;display:flex;flex-direction:column;gap:5px}.field label,.field legend{font-weight:670;font-size:12px}.field small{color:var(--muted);font-size:11px;line-height:1.4}.field input:not([type=checkbox]),.field select{width:100%;min-height:40px;border:1px solid #3c4f5f;background:#101c25;color:var(--text);border-radius:7px;padding:7px 9px}.field input[aria-invalid=true]{border-color:var(--red)}.field input:disabled,.field select:disabled{opacity:.7}.checks{border:0;padding:0;margin:0;grid-column:1/-1}.checkrow{display:flex;flex-wrap:wrap;gap:8px;margin-top:7px}.checkrow label{display:flex;align-items:center;gap:7px;padding:6px 9px;min-height:38px;background:#101c25;border:1px solid var(--line);border-radius:7px}.checkrow input{width:16px;height:16px;accent-color:var(--green);margin:0}.form-actions{display:flex;flex-wrap:wrap;align-items:center;gap:8px;padding:14px 18px}.btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;min-height:39px;padding:7px 12px;border:1px solid #415465;border-radius:8px;color:var(--text);background:#1c2b38;font-weight:670}.btn .icon{width:15px;height:15px}.btn:hover:not(:disabled){filter:brightness(1.16)}.btn.primary{color:#162313;background:var(--green);border-color:var(--green)}.form-message{margin:0 18px 14px;padding:9px 10px;background:#202f3a;border-radius:7px;color:#e0edf6;font-size:12px;overflow-wrap:anywhere}.form-message.bad{background:#392726;color:#ffd3cc}.form-message.good{background:#223127;color:#d5f7b0}.note{color:var(--muted);font-size:12px;margin:0}.note+.note{margin-top:11px}.preparing{padding:24px}.sys-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:13px}.sys-grid section:last-child{grid-column:1/-1}.sys-call{display:flex;align-items:flex-start;gap:10px}.sys-call .icon{color:var(--amber);margin-top:2px}.right{margin-left:auto}
dialog{width:min(420px,calc(100% - 28px));padding:19px;border-radius:13px;border:1px solid #456050;background:#192833;color:var(--text);box-shadow:0 24px 100px #0009}dialog::backdrop{background:#091119c9}dialog h2{font-size:18px;margin:0 0 6px}dialog p{color:var(--muted);margin:0 0 15px}dialog .form-actions{padding:0;justify-content:flex-end}
@media(max-width:1120px){:root{--sidew:216px}.main{padding-left:21px;padding-right:21px}.topbar{padding:0 21px}.grid{gap:8px}.sensor{padding:11px}.hero-art{right:14%}}
@media(max-width:940px){.grid{grid-template-columns:repeat(2,minmax(0,1fr));gap:11px}.reading{font-size:32px}.hero{min-height:204px}.hero-art{opacity:.39}.twocol,.settings-layout{grid-template-columns:1fr}.context{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:700px){:root{--sidew:248px}.sidebar{transform:translateX(-102%);transition:transform .2s ease;visibility:hidden}.sidebar.open{transform:none;visibility:visible}.shade.open{display:block;position:fixed;z-index:4;inset:0;background:#071018bd;border:0;width:100%}.shell{margin-left:0}.menu{display:inline-grid;place-items:center;min-width:42px;min-height:42px;padding:0}.topbar{height:61px;padding:0 14px}.top-ip,.crumb .scope,.crumb>.icon{display:none}.main{padding:16px 14px 25px}.hero{padding:18px 15px;display:block}.hero h1{font-size:35px}.hero-art{width:53%;right:-5%;top:38%;opacity:.35}.hero-foot{margin-top:13px}.hero-stats{width:auto;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));margin-top:16px;padding:6px 8px}.hstat{padding:3px 6px;display:flex;flex-direction:column;align-items:flex-start;gap:1px}.hstat+.hstat{border-top:0;border-left:1px solid #3b5345}.hstat b{font-size:10px}.sec-head{margin-top:22px}.sensor-foot a{display:inline-flex;align-items:center;min-height:34px}.pagehead{padding:8px 0 18px}}
@media(max-width:380px){.sensor{padding:9px}.sensor .reading{font-size:24px}.sensor-top .pin{letter-spacing:0}.relay-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.fields,.context,.sys-grid{grid-template-columns:1fr}.page-footer{flex-wrap:wrap}.sec-head .micro{font-size:9px}.hero h1{font-size:31px}}
@media(prefers-reduced-motion:reduce){*,*::before,*::after{transition:none!important;scroll-behavior:auto!important}}
</style>
</head>
<body>
<a class="skip" href="#content">Skip to content</a>
<button class="shade" id="shade" tabindex="-1" aria-label="Close navigation"></button>
<aside class="sidebar" id="sidebar">
<a href="#dashboard" class="brand"><span class="brandmark" data-icon="node"></span><span><strong>Field Node</strong><span class="micro">ENVIRONMENT CONSOLE</span></span></a>
<nav aria-label="Main navigation" id="navigation"></nav>
<div class="side-footer"><div class="side-device"><span data-icon="wifi"></span><div><strong>Device access point</strong><span>192.168.4.1 · No internet needed</span></div></div></div>
</aside>
<div class="shell" id="shell">
<header class="topbar"><div class="crumb"><button type="button" class="btn menu" id="menu" aria-label="Open navigation" aria-controls="sidebar" aria-expanded="false" data-icon="menu"></button><span class="scope">Workspace</span><span data-icon="chev"></span><strong id="crumb">Overview</strong></div><div class="top-right"><span class="top-ip">192.168.4.1</span><span id="link" class="pill" role="status"><i class="dot"></i><span>Connecting</span></span></div></header>
<main class="main" id="content" tabindex="-1">
<div id="banner" class="banner" hidden></div>
<div id="view"></div>
<noscript><p class="banner">Enable JavaScript in your browser to view readings and edit settings.</p></noscript>
<footer class="page-footer"><span>FIELD NODE &nbsp; / &nbsp; LOCAL DEVICE CONSOLE</span><span id="version">ESP32 · AP mode</span></footer>
</main>
</div>
<dialog id="confirmation" aria-labelledby="confirm-title" aria-describedby="confirm-text"><h2 id="confirm-title">Confirm action</h2><p id="confirm-text"></p><form class="form-actions" method="dialog"><button class="btn" type="submit" value="cancel" autofocus>Cancel</button><button class="btn primary" id="confirm-action" type="submit" value="ok">Continue</button></form></dialog>
<script>
'use strict';
const $=(s,r=document)=>r.querySelector(s), $$=(s,r=document)=>[...r.querySelectorAll(s)];
const paths={node:'<path d="M12 2v5m0 10v5M2 12h5m10 0h5M5 5l3.5 3.5m7 7L19 19m0-14-3.5 3.5m-7 7L5 19"/><circle cx="12" cy="12" r="4"/>',grid:'<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',activity:'<path d="M2 12h5l3-7 4 14 3-7h5"/>',display:'<rect x="3" y="4" width="18" height="13" rx="2"/><path d="M8 21h8m-4-4v4M7 11h4l2 2 4-4"/>',temp:'<path d="M9 14.8V5a3 3 0 0 1 6 0v9.8a5 5 0 1 1-6 0ZM12 7v9"/><circle cx="12" cy="18" r="1"/>',gas:'<path d="M12 22a7 7 0 0 0 7-7c0-4-3-6-4-9-1 2-2 3-3 3-1-2-1-4 0-7-5 4-7 9-7 13a7 7 0 0 0 7 7Z"/><path d="M12 22a3 3 0 0 0 3-3c0-2-2-3-3-5-1 2-3 3-3 5a3 3 0 0 0 3 3Z"/>',sun:'<circle cx="12" cy="12" r="4"/><path d="M12 2v2m0 16v2M2 12h2m16 0h2M5 5l1.5 1.5m11 11L19 19m0-14-1.5 1.5m-11 11L5 19"/>',motion:'<circle cx="12" cy="12" r="2"/><path d="M7 7a7 7 0 0 0 0 10m10 0a7 7 0 0 0 0-10M4 4a11 11 0 0 0 0 16m16 0a11 11 0 0 0 0-16"/>',bell:'<path d="M18 8a6 6 0 0 0-12 0c0 7-3 7-3 9h18c0-2-3-2-3-9m-8 12a2.4 2.4 0 0 0 4 0"/>',led:'<path d="M9 18h6m-5 3h4M9 16v-2a5 5 0 1 1 6 0v2m4-6h2M3 10h2"/>',relay:'<path d="M3 7h7l4 10h7M3 17h7m4-10h7"/><circle cx="3" cy="7" r="1"/><circle cx="21" cy="17" r="1"/>',wifi:'<path d="M3 8a14 14 0 0 1 18 0M6 12a9 9 0 0 1 12 0m-9 4a4.5 4.5 0 0 1 6 0m-3 4h.01"/>',chev:'<path d="m9 6 6 6-6 6"/>',menu:'<path d="M4 6h16M4 12h16M4 18h16"/>',shield:'<path d="M12 22s8-4 8-12V5l-8-3-8 3v5c0 8 8 12 8 12Z"/><path d="m9 12 2 2 4-4"/>',chip:'<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9" y="9" width="6" height="6" rx="1"/><path d="M9 3v3m6-3v3m-6 12v3m6-3v3M3 9h3m-3 6h3m12-6h3m-3 6h3"/>',save:'<path d="M5 3h12l4 4v12a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2Z"/><path d="M7 3v6h9V3M7 21v-8h10v8"/>',alert:'<path d="M12 3 2 20h20L12 3Zm0 6v5m0 3h.01"/>'};
const icon=(key)=>`<svg class="icon" viewBox="0 0 24 24" aria-hidden="true">${paths[key]||''}</svg>`;
$$('[data-icon]').forEach(el=>el.innerHTML=icon(el.dataset.icon));
const bySensor=['dht','mq2','ldr','pir'], names=['DHT22','MQ-2','LDR','PIR'];
const nav=[['dashboard','Overview','grid',''],['system','System status','activity',''],['oled','OLED display','display','I²C'],['dht','DHT22 climate','temp','04'],['mq2','MQ-2 gas','gas','35'],['ldr','LDR light','sun','34'],['pir','PIR motion','motion','27'],['buzzer','Buzzer','bell','26'],['led','LED indicator','led','33'],['relay','Relay routing','relay',''],['wifi','Wi-Fi access point','wifi','AP']];
$('#navigation').innerHTML=nav.map(([id,label,ico,port],i)=>(i===0?'<p class="side-label">Workspace</p>':i===2?'<p class="side-label">Settings management</p>':'')+`<a class="navlink" href="#${id}" data-page="${id}">${icon(ico)}<span>${label}</span>${port?`<small>${port}</small>`:''}</a>`).join('');
const on=['Disabled','Enabled'], rules=['Off','At or above threshold','At or below threshold'], routes=['None','Unified only','Individual only','Unified + individual'];
const select=(key,label,options,hint='',offset=0)=>({key,label,options,hint,offset});
const num=(key,label,min,max,step=1,hint='')=>({key,label,min,max,step,hint});
const text=(key,label,max,hint='',password=false)=>({key,label,max,hint,text:true,password});
const mask=(key,label)=>({key,label,mask:true,hint:'Only selected modules can activate this output.'});
const routeField=(prefix)=>select(prefix+'_relay','Relay route',routes,'Limited by the global setting in Relay routing.');
const groups={
 oled:{title:'OLED display',desc:'Choose what the local 128 × 64 display shows and adjust how it reads at a glance.',port:'SDA GPIO 21 · SCL GPIO 22',driver:'SSD1306 · I²C',notes:['Use the address printed on your module or confirmed with an I²C scan. Most are 0x3C.','Unavailable readings appear as dashes. The OLED is probed automatically after a detected disconnect.'],sections:[['Screen controls','Adjust the physical display.',[select('oled_on','Display',on),select('oled_addr','I²C address',['0x3C','0x3D'],'',60),num('oled_contrast','Contrast',1,255),select('oled_rotate','Orientation',['Normal','Rotated 180°'])]],['Content and timing','Continuous display refresh without changing the dashboard update rate.',[select('oled_page','Screen view',['Alternate pages','Sensor readings','System status']),num('oled_ms','Refresh interval (ms)',500,10000)]]]},
 dht:{title:'DHT22 climate',desc:'Configure temperature and humidity alarms independently. Either rule can activate this module’s outputs.',port:'GPIO 4 · Digital data',driver:'DHT22 · Temperature + RH',notes:['Use a data pull-up to 3.3 V if your module does not include one.','A failed or stale sample clears this module’s alarms. Hysteresis must leave a clear point inside the sensor range.'],sections:[['Acquisition','The DHT22 cannot be sampled faster than its minimum settling time.',[select('dht_on','Climate monitoring',on),num('dht_ms','Sample interval (ms)',2500,60000)]],['Temperature alarm','Choose the trigger direction and the gap used to clear an active alarm.',[select('temp_mode','Temperature rule',rules),num('temp_limit','Temperature threshold (°C)',-40,80,.1),num('temp_hys','Temperature hysteresis (°C)',0,30,.1)]],['Humidity alarm','Relative humidity is expressed as a percentage.',[select('hum_mode','Humidity rule',rules),num('hum_limit','Humidity threshold (%)',0,100,.1),num('hum_hys','Humidity hysteresis (%)',0,30,.1)]],['Output routing','Temperature and humidity share one DHT22 relay channel.',[routeField('dht')]]]},
 mq2:{title:'MQ-2 gas sensor',desc:'Set the raw analog trigger and the warm-up delay before readings can raise an alarm.',port:'GPIO 35 · ADC1',driver:'MQ-2 · 12-bit raw ADC',notes:['Use a divider so the analog input never exceeds safe ESP32 limits. Power the heater from a suitable supply.','ADC counts are not ppm. Follow the module’s burn-in instructions. This device is not a certified gas or smoke alarm.'],sections:[['Acquisition','During warm-up, the reading is withheld and the relay remains inactive.',[select('gas_on','Gas monitoring',on),num('gas_warm','Warm-up (seconds)',30,3600)]],['Gas alarm','Choose a threshold after observing your installed sensor.',[select('gas_mode','Gas rule',rules),num('gas_limit','Threshold (ADC counts)',0,4095),num('gas_hys','Hysteresis (ADC counts)',0,4095),routeField('gas')]]]},
 ldr:{title:'LDR light sensor',desc:'Set an analog light threshold for the orientation of your LDR voltage divider.',port:'GPIO 34 · ADC1',driver:'LDR · 12-bit raw ADC',notes:['The divider can make the reading rise or fall with increasing light. Observe both lighting conditions before choosing a rule.','Raw ADC counts are not lux. A disconnected analog input can still return a plausible count.'],sections:[['Acquisition','The dashboard shows the median of the latest five ADC samples.',[select('light_on','Light monitoring',on)]],['Light alarm','Use hysteresis to prevent repeated switching near the threshold.',[select('light_mode','Light rule',rules),num('light_limit','Threshold (ADC counts)',0,4095),num('light_hys','Hysteresis (ADC counts)',0,4095),routeField('light')]]]},
 pir:{title:'PIR motion sensor',desc:'Set the start-up stabilization time and how long an alarm holds after motion stops.',port:'GPIO 27 · Digital input',driver:'PIR · Active HIGH',notes:['Verify the PIR module’s output voltage and common ground before connecting it.','The hold time starts after the last debounced HIGH. A module’s own hold time may add to the total motion duration.'],sections:[['Acquisition','The PIR signal is debounced before it can trigger an alarm.',[select('pir_on','Motion monitoring',on),num('pir_warm','Warm-up (seconds)',0,300)]],['Motion alarm','The hold is only applied when the motion alarm is enabled.',[select('pir_alarm','Alarm on motion',on),num('pir_hold','Alarm hold (seconds)',0,600),routeField('pir')]]]},
 buzzer:{title:'Buzzer',desc:'Choose which module alarms may start the active buzzer and set its pulse pattern.',port:'GPIO 26 · Active HIGH',driver:'Active buzzer · Digital output',notes:['Choose an active buzzer or a suitable driver. A passive transducer requires a tone signal and is not supported here.','The pulse restarts when an alarm-matched buzzer goes from idle to active. The output stops when all selected alarms clear.'],sections:[['Source and timing','A pulse equal to its cycle length produces a steady output.',[select('buzz_on','Buzzer output',on),num('buzz_period','Cycle length (ms)',200,10000),num('buzz_pulse','HIGH pulse (ms)',50,10000,1,'Must not exceed cycle length.'),mask('buzz_mask','Allowed alarm sources')]]]},
 led:{title:'LED indicator',desc:'Use the indicator as a steady or blinking signal for one or more module alarms.',port:'GPIO 33 · Active HIGH',driver:'LED anode · Digital output',notes:['Connect the anode through a current-limiting resistor and the cathode to ground.','The blink uses equal ON and OFF times. The LED switches off as soon as all selected alarms clear.'],sections:[['Source and timing','A cycle of zero means steady. Otherwise the cycle must be at least 200 ms.',[select('led_on','LED output',on),num('led_period','Blink cycle (ms)',0,10000),mask('led_mask','Allowed alarm sources')]]]},
 relay:{title:'Relay routing',desc:'Control whether alarms drive the shared relay, individual relays, or both.',port:'GPIO 25 · 15 · 18 · 19 · 5',driver:'Five channels · Active HIGH',notes:['Unified GPIO 25 uses OR logic across routed, active alarms. GPIOs 15, 18, 19 and 5 carry DHT22, MQ-2, LDR and PIR respectively.','GPIO 5 and 15 are boot strapping pins. Use high-impedance drivers and a hardware output inhibit where outputs must stay off before firmware starts. Never drive bare coils from GPIO.'],sections:[['Global mode','Individual route values are intersected with the global mode.',[select('relay_mode','Relay operation',routes)]],['Module routes','These values are shared with the four sensor pages.',[select('dht_relay','DHT22 / GPIO 15',routes),select('gas_relay','MQ-2 / GPIO 18',routes),select('light_relay','LDR / GPIO 19',routes),select('pir_relay','PIR / GPIO 5',routes)]]]},
 wifi:{title:'Wi-Fi access point',desc:'Manage the device’s local Wi-Fi network. Internet access is not needed to use this console.',port:'192.168.4.1 · IPv4',driver:'ESP32 · 2.4 GHz WPA2',notes:['The setup password is 12345678. Change it before deployment and share the new password only with trusted operators. Anyone connected to this AP can manage the device.','A Wi-Fi change saves first and reboots about ten seconds later. Reconnect to the new network and open http://192.168.4.1/ if the captive prompt does not appear.'],sections:[['Network credentials','The current password is never shown. Leave the new password blank to keep it.',[text('ssid','Network name (SSID)',32,'1–32 printable ASCII characters.'),text('pass','New Wi-Fi password',63,'8–63 printable ASCII characters.',true)]],['Radio settings','The network is always visible for local setup.',[num('ap_chan','Channel',1,11),num('ap_max','Maximum clients',1,4)]]]},
};
const info=[['Temperature & humidity','temp','GPIO 04','% RH'],['Gas signal','gas','GPIO 35','ADC'],['Ambient light','sun','GPIO 34','ADC'],['Motion','motion','GPIO 27','']];
const outputs=[['Unified relay','GPIO 25'],['DHT22','GPIO 15'],['MQ-2','GPIO 18'],['LDR','GPIO 19'],['PIR','GPIO 05']];
const artwork=`<svg class="hero-art" viewBox="0 0 360 244" fill="none" aria-hidden="true"><g stroke="currentColor" opacity=".66"><path stroke-dasharray="3 6" d="M0 122h360M180 0v244m-92-45L273 46M90 45l180 154"/><circle cx="180" cy="122" r="101" stroke-dasharray="2 5"/><circle cx="180" cy="122" r="82"/><circle cx="180" cy="122" r="62"/><circle cx="180" cy="122" r="39" stroke-dasharray="4 5"/><path d="M180 21v-14m0 216v14M79 122H65m216 0h14M133 105v-25l-19-19H85l-18-18H24m201 97v25l19 19h41l18 18h40M98 170H70l-18 18H18M262 74h23l18-18h40"/><rect x="161" y="103" width="38" height="38" rx="8"/><path d="M169 122h6l4-8 4 16 4-8h5"/></g><g fill="currentColor"><circle cx="180" cy="40" r="3"/><circle cx="242" cy="122" r="3"/><circle cx="108" cy="160" r="4"/><circle cx="246" cy="73" r="3"/><circle cx="52" cy="188" r="3"/><circle cx="303" cy="56" r="3"/><circle cx="24" cy="43" r="3"/><circle cx="343" cy="202" r="3"/></g></svg>`;
const esc=(s)=>String(s).replace(/[&<>"']/g,(c)=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[c]);
const tag=(s,kind='')=>`<span class="tag ${kind}"><i class="dot"></i>${esc(s)}</span>`;
const rows=(items)=>`<ul class="rows">${items.map(([k,v])=>`<li><span>${esc(k)}</span><b>${esc(v)}</b></li>`).join('')}</ul>`;
const panel=(t,sub,body,extra='')=>`<section class="panel"><div class="panel-head"><div><h2>${t}</h2>${sub?`<p>${sub}</p>`:''}</div>${extra}</div><div class="panel-body">${body}</div></section>`;
let page='', status=null, online=false, failures=0, dirty=false, saving=false, generation=0, lastRead=0, polling=false, allowHash=false, open=false, currentConfig=null;
let formRequest=null;
const mobile=matchMedia('(max-width: 700px)');
const readable={ready:'Ready',warming:'Warming up',fault:'Read error',disabled:'Disabled'};
const storage={starting:'Starting',defaults:'Factory defaults',healthy:'Record verified',recovered:'Recovered copy',error:'Storage error'};
const resetReasons={0:'Unknown',1:'Power on',2:'External reset',3:'Software reset',4:'Panic',5:'Interrupt watchdog',6:'Task watchdog',7:'Other watchdog',8:'Deep sleep',9:'Brownout',10:'SDIO'};
const take=(m,k)=>!!(m&(1<<k));
const fresh=()=>online&&status;
const val=(f)=>fresh()?f(status):'—';
function age(n){try{const s=BigInt(n);if(s<60n)return s+'s';const d=s/86400n,h=s/3600n%24n,m=s/60n%60n;return d?`${d}d ${h}h`:h?`${h}h ${m}m`:`${m}m ${s%60n}s`;}catch{return '—';}}
const bytes=(n)=>`${Math.round(n/1024)} KiB`;
function setDrawer(opened,restore=false){
 open=opened&&mobile.matches;
 $('#sidebar').classList.toggle('open',open);$('#shade').classList.toggle('open',open);
 $('#menu').setAttribute('aria-expanded',String(open));
 $('#sidebar').inert=mobile.matches&&!open;$('#shell').inert=open;
 if(open){($('#sidebar [aria-current="page"]')||$('.brand')).focus();}else if(restore){$('#menu').focus();}
}
$('#menu').onclick=()=>setDrawer(true);$('#shade').onclick=()=>setDrawer(false,true);
const drawerMode=()=>setDrawer(false);
if(mobile.addEventListener)mobile.addEventListener('change',drawerMode);else mobile.addListener(drawerMode);
drawerMode();
document.addEventListener('keydown',(e)=>{
 if(!open||$('#confirmation').open)return;
 if(e.key==='Escape'){e.preventDefault();setDrawer(false,true);}
 if(e.key==='Tab'){const links=$$('#sidebar a'),first=links[0],last=links[links.length-1];if((!e.shiftKey&&document.activeElement===last)||(e.shiftKey&&document.activeElement===first)){e.preventDefault();(e.shiftKey?last:first).focus();}}
});
function heading(code,title,description){return `<header class="pagehead"><div class="eyebrow"><i class="dot"></i>${code}</div><h1>${title}</h1><p>${description}</p></header>`;}
function mark(el,s,kind){const cls='pill '+kind;if(el.className!==cls)el.className=cls;if(el.lastElementChild.textContent!==s)el.lastElementChild.textContent=s;}
function globalUpdate(){
 mark($('#link'),window.fieldDemo?'Preview · simulated':online?'Local connection':failures?'Disconnected':'Connecting',online?'good':failures?'bad':'');
 $('#version').textContent=status?`ESP32 · Firmware ${status.version}`:'ESP32 · AP mode';
 const b=$('#banner');let message='';
 if(failures&&!online)message=window.fieldDemo?'The preview is temporarily unavailable.':'Connection lost. Reconnect to the device’s Wi-Fi, then keep this page open or try http://192.168.4.1/.';
 else if(fresh()&&status.restarting)message=(status.storage==='error'?'The latest write was not verified. ':'')+(status.restartIn?`Device restart in about ${status.restartIn}s.`:'Restart pending. Release the BOOT button if it is held.');
 else if(fresh()&&(status.inhibit||status.workerFault))message='Outputs are inhibited. Check System status.';
 else if(fresh()&&status.storage==='error')message='Configuration storage reports an error. Check System status before relying on the last saved settings.';
 const html=message?`<span>${esc(message)}</span>${page==='system'?'':'<a href="#system">System status</a>'}`:'';
 if(b.innerHTML!==html)b.innerHTML=html;b.hidden=!message;
}
function dashboard(){
 $('#view').innerHTML=`<section class="hero">${artwork}<div class="hero-copy"><div class="eyebrow"><i class="dot"></i> LOCAL ENVIRONMENT MONITOR</div><h1>Sensor overview</h1><p>Live readings, alarm conditions and output status from your ESP32 field node.</p><div class="hero-foot"><span class="micro">${icon('chip')} ESP32 FIELD UNIT</span><span class="micro">${icon('shield')} LOCAL ACCESS ONLY</span></div></div><div class="hero-stats" id="hero-stats"></div></section>
 <div class="sec-head"><div><h2>Live sensors</h2><p>Current readings from the four monitored modules.</p></div><div class="micro" id="sample">Awaiting telemetry</div></div>
 <div class="grid" id="sensor-grid">${info.map(([title,ico,pin],i)=>`<article class="sensor" data-s="${i}"><div class="sensor-top"><span class="sensor-icon">${icon(ico)}</span><span class="pin">${pin}</span></div><h3>${title}</h3><div class="reading">—</div><p class="subread">Awaiting telemetry</p><div class="meter" aria-hidden="true"><span></span></div><div class="sensor-foot"><span class="sensor-tag"></span><a href="#${bySensor[i]}" aria-label="Settings for ${title}">Settings ${icon('chev')}</a></div></article>`).join('')}</div>
 <div class="sec-head"><div><h2>Device outputs</h2><p>GPIO commands and indicators driven by the configured alarm rules.</p></div><a class="textlink" href="#relay">Configure routing ${icon('chev')}</a></div>
 <div class="twocol">${panel('Relay matrix','Shared + four individual channels','<div class="relay-grid" id="relays"></div>',`<span class="accent">${icon('relay')}</span>`)}${panel('Indicator state','Live logic levels · not load feedback','<div id="indicators"></div>',`<span class="accent">${icon('activity')}</span>`)}</div>`;
 updateDashboard();
}
function updateDashboard(){
 const s=fresh(), live=s?.state?.filter(x=>x==='ready').length;
 $('#hero-stats').innerHTML=`<div class="hstat"><span>Ready modules</span><b class="good">${s?`${live} / 4`:'—'}</b></div><div class="hstat"><span>Active alarms</span><b>${s?names.filter((_,i)=>take(s.alarms,i)).length:'—'}</b></div><div class="hstat"><span>Device uptime</span><b>${val(x=>age(x.uptime))}</b></div>`;
 $('#sample').textContent=s?`Updated ${new Date(lastRead).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'})}`:failures?'Telemetry offline':'Awaiting telemetry';
 $$('.sensor').forEach((el,i)=>{
   const st=s?.state?.[i], valid=st==='ready', alarm=s&&take(s.alarms,i);let reading='—',unit='', detail='Awaiting telemetry',pct=0;
   if(valid){if(i===0){reading=(s.temp/10).toFixed(1);unit='°C';detail=`Humidity ${(s.hum/10).toFixed(1)}% RH`;pct=s.hum/10;}
    if(i===1||i===2){reading=i===1?s.gas:s.light;unit='ADC';pct=reading/4095*100;detail='Median raw count · 0–4095';}
    if(i===3){reading=s.pir?'Motion':s.held?'Hold':'Clear';pct=s.pir||s.held?100:0;detail=s.pir?'Motion detected':s.held?'Alarm hold active':'No motion detected';}}
   else if(st==='warming'){const warm=i===1?s.warmGas:i===3?s.warmPir:0;detail=warm?`${warm}s until ready`:'Waiting for a valid reading';}
   else if(st==='disabled')detail=s.inhibit?'Outputs inhibited':'Monitoring turned off';
   else if(st==='fault')detail=i===0?'No valid DHT22 sample':'ADC read unavailable';else if(failures)detail='No current device data';
   $('.reading',el).innerHTML=`${esc(reading)}${unit?`<span class="unit">${unit}</span>`:''}`;
   $('.subread',el).textContent=detail;$('.meter span',el).style.width=`${Math.max(0,Math.min(100,pct))}%`;
   $('.sensor-tag',el).innerHTML=tag(alarm?'Alarm':readable[st]||'Unknown',alarm?'warn':st==='ready'?'good':st==='fault'?'bad':st==='warming'?'warn':'');
 });
 $('#relays').innerHTML=outputs.map(([name,pin],i)=>`<div class="relay-cell"><div><b>${name}</b><small>${pin}</small></div>${s?tag(take(s.outputs,i)?'ON':'OFF',take(s.outputs,i)?'good':''):tag('—')}</div>`).join('');
 $('#indicators').innerHTML=rows([['Buzzer · GPIO 26',val(x=>take(x.outputs,5)?'HIGH':'LOW')],['LED · GPIO 33',val(x=>take(x.outputs,6)?'HIGH':'LOW')],['Relay mode',val(x=>routes[x.relayMode])]]);
}
function system(){
 $('#view').innerHTML=heading('DEVICE HEALTH','System status','Check the device’s health, active network and configuration storage.')+'<div class="sys-grid" id="sys"></div>';updateSystem();
}
function updateSystem(){
 const p=(title,desc,entries,ico)=>panel(title,desc,rows(entries),`<span class="accent">${icon(ico)}</span>`);
 $('#sys').innerHTML=p('Runtime','Control loop and heap headroom',[['Firmware',val(x=>x.version)],['Uptime',val(x=>age(x.uptime))],['Reset reason',val(x=>resetReasons[x.bootReason]||`Code ${x.bootReason}`)],['Free heap',val(x=>bytes(x.heap))],['Low-water heap',val(x=>bytes(x.minHeap))],['Largest internal block',val(x=>bytes(x.largest))],['Workers',val(x=>x.workerFault?'Startup failed':'No startup failure')],['Outputs',val(x=>x.inhibit?'Inhibited':'Normal')]],'chip')+
 p('Connectivity & persistence','Local AP and saved configuration',[['Active SSID',val(x=>x.ssid)],['Saved SSID',val(x=>x.savedSsid??x.ssid)],['Portal IP','192.168.4.1'],['Connected stations',val(x=>x.stations)],['Storage',val(x=>storage[x.storage])],['Config revision',val(x=>x.rev)],['Last settings transaction',val(x=>x.save)],['Save cooldown',val(x=>x.cooldown?age(x.cooldown):'Ready')],['OLED',val(x=>x.oled)],['DHT22 failed reads',val(x=>x.dhtFailures)]],'wifi')+
 panel('Factory reset','Physical confirmation only',`<div class="sys-call">${icon('alert')}<p class="note">With the device running, press and hold the onboard BOOT button for at least five seconds, then release it. Sensor alerts and outputs switch off while factory settings are stored. After restart, reconnect to ESP32_Multi_Sensors using the default setup password. If a storage error is reported, the reset may not have been written.</p></div>`);
}
function fieldMarkup(f){
 const id=`field-${f.key}`,hint=f.hint?`<small id="${id}-hint">${esc(f.hint)}</small>`:'',aria=f.hint?`aria-describedby="${id}-hint"`:'';
 if(f.mask)return `<fieldset class="field checks" ${aria}><legend>${f.label}</legend>${hint}<div class="checkrow">${names.map((n,i)=>`<label><input type="checkbox" data-mask="${f.key}" value="${i}">${n}</label>`).join('')}</div></fieldset>`;
 const element=f.options?`<select id="${id}" name="${f.key}" ${aria}>${f.options.map((n,i)=>`<option value="${i+f.offset}">${n}</option>`).join('')}</select>`:f.text?`<input id="${id}" name="${f.key}" ${aria} type="${f.password?'password':'text'}" maxlength="${f.max}" minlength="${f.password?8:1}" pattern="[ -~]+" spellcheck="false" autocapitalize="none" autocomplete="${f.password?'new-password':'off'}" ${f.password?'':'required'}>`:`<input id="${id}" name="${f.key}" ${aria} type="number" min="${f.min}" max="${f.max}" step="${f.step}" required>`;
 return `<div class="field"><label for="${id}">${f.label}</label>${element}${hint}</div>`;
}
function details(group){
 const s=fresh(),index=bySensor.indexOf(page);let items=[['Interface',group.driver],['Pins / address',group.port],['Config revision',val(x=>x.rev)]];
 if(index>=0)items.push(['Monitoring',s?readable[s.state[index]]:'—'],['Alarm',s?take(s.alarms,index)?'Active':'Clear':'—']);
 if(page==='oled')items.push(['Display response',val(x=>x.oled)]);
 if(page==='buzzer'||page==='led')items.push(['Command level',val(x=>take(x.outputs,page==='buzzer'?5:6)?'HIGH':'LOW')]);
 if(page==='relay')items.push(['Global route',val(x=>routes[x.relayMode])]);
 if(page==='wifi')items.push(['Connected stations',val(x=>x.stations)]);
 $('#detail').innerHTML=rows(items);
}
function message(text='',kind='') {const m=$('#message');if(m){m.textContent=text;m.hidden=!text;m.className=`form-message ${kind}`;}}
function markForm(){
 const form=$('#settings-form');if(!form)return;
 const fieldset=$('#edit-fields'),reload=$('#reload'),save=$('#save');
 fieldset.disabled=!currentConfig||saving;
 reload.disabled=saving;
 save.disabled=!currentConfig||saving||!online;
}
const delay=(ms)=>new Promise(resolve=>setTimeout(resolve,ms));
function ask(title,body,verb){
 const dialog=$('#confirmation');if(dialog.open)return Promise.resolve(false);
 $('#confirm-title').textContent=title;$('#confirm-text').textContent=body;$('#confirm-action').textContent=verb;
 dialog.returnValue='cancel';dialog.showModal();
 return new Promise(resolve=>dialog.addEventListener('close',()=>resolve(dialog.returnValue==='ok'),{once:true}));
}
async function api(path,options={}){
 if(window.fieldDemo)return window.fieldDemo(path,options);
 const controller=new AbortController(), timeout=setTimeout(()=>controller.abort(),3500);
 try{
  const response=await fetch(path,{...options,cache:'no-store',signal:controller.signal});
  const data=await response.json();if(!response.ok)throw Object.assign(new Error(data.error||`Device returned HTTP ${response.status}.`),{http:response.status});
  return data;
 }catch(e){if(e.name==='AbortError')throw new Error('No timely response from the device.');throw e;}finally{clearTimeout(timeout);}
}
function populate(config,group){
 for(const [, ,fields] of group.sections)for(const f of fields){
   if(f.mask){$$(`[data-mask="${f.key}"]`).forEach(el=>{el.checked=take(config.settings[f.key],+el.value);});continue;}
   const el=$(`#field-${f.key}`);el.setCustomValidity('');el.removeAttribute('aria-invalid');
   el.value=f.key==='ssid'?config.ssid:f.key==='pass'?'':f.step===.1?config.settings[f.key]/10:config.settings[f.key];
 }
}
async function loadForm(group,force=false){
 if(saving)return;
 const ticket=generation;
 if(force&&dirty&&!(await ask('Reload saved settings?','The unsaved changes on this page will be discarded.','Reload')))return;
 if(ticket!==generation||saving)return;
 if(force)dirty=false;
 const request=Symbol();formRequest=request;currentConfig=null;markForm();message('Loading saved settings…');
 try{
  const config=await api('/api/config');if(ticket!==generation||formRequest!==request)return;
  currentConfig=config;populate(config,group);message();
 }catch(e){if(ticket!==generation||formRequest!==request)return;message(e.message+' Reconnect if needed and select Reload saved.','bad');}
 markForm();
}
function validate(form,params){
 const bad=(name,error)=>{const f=form.elements.namedItem(name);f.setCustomValidity(error);f.setAttribute('aria-invalid','true');f.reportValidity();message(error,'bad');return false;};
 if(page==='wifi'&&!params.get('ssid').trim())return bad('ssid','Enter a network name with at least one visible character.');
 if(page==='buzzer'&&+params.get('buzz_pulse')>+params.get('buzz_period'))return bad('buzz_pulse','The HIGH pulse cannot exceed the cycle length.');
 const led=+params.get('led_period');if(page==='led'&&led!==0&&led<200)return bad('led_period','Enter zero for a steady LED, or at least 200 ms.');
 for(const [id,lo,hi] of [['temp',-400,800],['hum',0,1000],['gas',0,4095],['light',0,4095]]){
   const mode=+params.get(id+'_mode'), limit=+params.get(id+'_limit'),gap=+params.get(id+'_hys');
   if((mode===1&&limit-gap<lo)||(mode===2&&limit+gap>hi))return bad(id+'_hys','Choose a hysteresis that keeps the clear point within the sensor range.');
 }return true;
}
async function submit(e,group){
 e.preventDefault();const form=e.currentTarget;if(saving||!currentConfig||!online)return;
 const ticket=generation,base=currentConfig;
 const params=new URLSearchParams({rev:base.rev});
 for(const [,,fields] of group.sections)for(const f of fields){
   if(f.mask){let bits=0;$$(`[data-mask="${f.key}"]:checked`,form).forEach(x=>bits|=1<<+x.value);params.set(f.key,bits);}
   else {const item=form.elements.namedItem(f.key);params.set(f.key,f.step===.1?Math.round(+item.value*10):item.value);}
 }
 if(!validate(form,params))return;
 if(page==='wifi'&&!(await ask('Save Wi-Fi settings?','If Wi-Fi settings changed, the device will restart and you will need to reconnect.','Save settings')))return;
 if(ticket!==generation||base!==currentConfig||saving||!online)return;
 saving=true;markForm();message('Saving and verifying settings…');
 let submitted=false;
 try{
  const res=await api('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Field-Token':base.token},body:params.toString()});
  if(res.unchanged){dirty=false;populate(currentConfig,group);message('No changes to save.','good');return;}
  submitted=true;
  let verified=false;
  for(let i=0;i<15;i++){
   await delay(350);const job=await api('/api/status');
   if(job.job!==res.job)throw new Error('The device changed transactions before confirmation. Reload saved settings.');
   if(job.save==='failed')throw new Error('Flash verification failed. Previously active settings were kept.');
   if(job.save==='saved'){verified=true;break;}
  }
  if(!verified)throw new Error('The save is still unconfirmed. Reload saved settings before retrying.');
  dirty=false;
  if(res.wifi){currentConfig=null;form.elements.namedItem('pass').value='';message('Wi-Fi settings saved. Reconnect after the device restarts.','good');}
  else {
   message('Settings saved and verified.','good');
   try{currentConfig=await api('/api/config');populate(currentConfig,group);}catch{currentConfig=null;message('Settings saved. Select Reload saved before editing again.','good');}
  }
 }catch(e){message((submitted||e.http)?e.message:'Save could not be confirmed. Reload saved settings before retrying.','bad');if(!e.http&&e.message.indexOf('Flash verification failed')!==0)currentConfig=null;}
 finally{saving=false;markForm();}
}
function settings(group){
 $('#view').innerHTML=heading('SETTINGS MANAGEMENT',group.title,group.desc)+`<div class="settings-layout"><section class="panel form-panel"><div class="panel-head"><div><h2>Configuration</h2><p>Changes take effect after a verified save.</p></div><span class="accent">${icon('save')}</span></div><form id="settings-form"><fieldset id="edit-fields" style="border:0;padding:0;margin:0;min-width:0" disabled><legend class="sr">Device settings</legend>${group.sections.map(([title,desc,fields])=>`<section class="group"><h3>${title}</h3><p>${desc}</p><div class="fields">${fields.map(fieldMarkup).join('')}</div></section>`).join('')}</fieldset><div class="form-actions"><button class="btn primary" id="save" type="submit" disabled>${icon('save')} Save changes</button><button class="btn" type="button" id="reload">Reload saved</button></div><p id="message" class="form-message" role="status" aria-live="polite" hidden></p></form></section><aside class="context">${panel('Module details','','<div id="detail"></div>',`<span class="accent">${icon('chip')}</span>`)}${panel('Installation notes','',group.notes.map(s=>`<p class="note">${esc(s)}</p>`).join(''),`<span class="accent">${icon('alert')}</span>`)}</aside></div>`;
 const form=$('#settings-form');form.addEventListener('input',()=>{dirty=true;$$('[aria-invalid=true]',form).forEach(x=>{x.setCustomValidity('');x.removeAttribute('aria-invalid');});message();});
 form.addEventListener('submit',(e)=>submit(e,group));$('#reload').onclick=()=>loadForm(group,true);
 details(group);loadForm(group);
}
function render(id,focus){
 generation++;page=id;dirty=false;currentConfig=null;setDrawer(false);
 $$('[data-page]').forEach(a=>{if(a.dataset.page===id)a.setAttribute('aria-current','page');else a.removeAttribute('aria-current');});
 const label=nav.find(n=>n[0]===id)[1];$('#crumb').textContent=label;document.title=`${label} · Field Node`;
 if(id==='dashboard')dashboard();else if(id==='system')system();else settings(groups[id]);
 globalUpdate();if(focus){window.scrollTo(0,0);$('#content').focus({preventScroll:true});}
}
function requested(){return nav.some(n=>n[0]===location.hash.slice(1))?location.hash.slice(1):'dashboard';}
const canLeave=()=>!saving&&!$('#confirmation').open&&(!dirty||ask('Discard unsaved changes?','Your edits will not be saved to the device.','Discard changes'));
window.addEventListener('hashchange',async ()=>{
 if(allowHash){allowHash=false;return;}
 const id=requested();if(id===page)return;
 if(await canLeave())render(id,true);else {try{history.replaceState(null,'',`#${page}`);}catch{allowHash=true;location.hash=page;}}
});
document.addEventListener('click',async e=>{
 const a=e.target.closest('a[href^="#"]');if(!a||e.defaultPrevented||e.ctrlKey||e.metaKey||e.shiftKey||e.altKey)return;
 const hash=a.getAttribute('href'),id=hash.slice(1);if(id==='content'){e.preventDefault();$('#content').focus();return;}
 if(!nav.some(n=>n[0]===id))return;
 e.preventDefault();if(id===page){if(open)setDrawer(false,true);return;}if(!(await canLeave()))return;
 if(window.fieldDemo){try{history.pushState(null,'',hash);}catch{if(location.hash!==hash){allowHash=true;location.hash=hash;}}}else history.pushState(null,'',hash);
 render(id,true);
});
window.addEventListener('beforeunload',e=>{if(dirty||saving){e.preventDefault();e.returnValue='';}});
async function poll(){
 if(polling||document.hidden)return;polling=true;
 try{status=await api('/api/status');online=true;failures=0;lastRead=Date.now();}
 catch{failures++;online=false;}
 finally{
  polling=false;globalUpdate();
  if(page==='dashboard')updateDashboard();else if(page==='system')updateSystem();else if(groups[page]){details(groups[page]);markForm();}
 }
}
render(requested(),false);poll();setInterval(poll,2000);document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});
</script>
</body>
</html>
)FIELDUI";

// ========================================================
// Fixed-capacity HTTP and captive DNS
// ========================================================
constexpr size_t MaxHeader=2400,MaxBody=1600,Slots=5,WriteChunk=1460;
struct Request {
  char *method=nullptr,*path=nullptr,*host=nullptr,*origin=nullptr,*token=nullptr,*type=nullptr;
  size_t length=0;
  bool hasLength=false;
};
struct Client {
  Request parsed;
  int fd=-1;
  Ms idle=0,absolute=0;
  size_t used=0,split=0,len=0,headerLen=0,sent=0;
  bool sending=false,head=false;
  const char *body=nullptr;
  char request[MaxHeader+MaxBody+1];
  char header[720];
  char data[2304];
};
Client clients[Slots];
int tcp=-1,dns=-1;

bool retrySocket() { return errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR || errno==ENOBUFS || errno==ENOMEM; }

void closeClient(Client &c) {
  if (c.fd>=0) close(c.fd);
  c.fd=-1;
  memset(c.request,0,c.used);
  c.used=0;
}

void respond(Client &c,const char *code,const char *type,const char *body,size_t size,const char *extra="") {
  int written=snprintf(c.header,sizeof c.header,
    "HTTP/1.1 %s\r\nConnection: close\r\nCache-Control: no-store\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
    "X-Content-Type-Options: nosniff\r\nReferrer-Policy: no-referrer\r\nX-Frame-Options: DENY\r\n"
    "Content-Security-Policy: default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; form-action 'none'; base-uri 'none'; frame-ancestors 'none'\r\n%s\r\n",
    code,type,static_cast<unsigned>(size),extra);
  if (written<=0 || static_cast<size_t>(written)>=sizeof c.header) { closeClient(c); return; }
  c.headerLen=static_cast<size_t>(written); c.sent=0; c.body=body; c.len=c.head?0:size; c.sending=true;
}

void json(Client &c,const char *code,const char *body) { respond(c,code,"application/json",body,strlen(body)); }

void redirect(Client &c) {
  constexpr char Link[]="Location: http://192.168.4.1/\r\n";
  respond(c,"302 Found","text/plain; charset=utf-8","",0,Link);
}

struct Text {
  char *buf; size_t cap,pos=0; bool ok=true;
  void add(const char *fmt,...) __attribute__((format(printf,2,3))) {
    if (!ok) return;
    va_list ap; va_start(ap,fmt); const int n=vsnprintf(buf+pos,cap-pos,fmt,ap); va_end(ap);
    if (n<0 || static_cast<size_t>(n)>=cap-pos) { ok=false; return; }
    pos+=static_cast<size_t>(n);
  }
  void string(const char *s) {
    add("\"");
    for (;*s;++s) { if (*s=='\"' || *s=='\\') add("\\"); add("%c",*s); }
    add("\"");
  }
};

const char *boolean(bool b) { return b?"true":"false"; }

void getConfig(Client &c) {
  Config cfg; Meta m; snapshot(cfg,m);
  if (!m.ready) { json(c,"503 Service Unavailable",R"({"error":"Settings are loading. Try again in a moment."})"); return; }
  Text t{c.data,sizeof c.data};
  t.add("{\"rev\":\"%" PRIu64 "\",\"token\":\"%s\",\"ssid\":",m.rev,csrf);
  t.string(cfg.ssid);
  t.add(",\"settings\":{");
  for (size_t i=0;i<KeyCount;++i) t.add("%s\"%s\":%" PRId64,i?",":"",Specs[i].name,cfg.v[i]);
  t.add("}}");
  if (t.ok) json(c,"200 OK",c.data); else json(c,"500 Internal Server Error","{}");
}

void getStatus(Client &c) {
  View v; Meta m; Climate d; Config cfg;
  portENTER_CRITICAL(&guard); v=sensors; m=meta; d=climate; cfg=live; portEXIT_CRITICAL(&guard);
  const Ms now=nowMs();
  Text t{c.data,sizeof c.data};
  constexpr const char *readings[]={"disabled","warming","ready","fault"};
  constexpr const char *stores[]={"starting","defaults","healthy","recovered","error"};
  constexpr const char *results[]={"none","pending","saved","failed"};
  const Ms freeAt=saveAt(m);
  t.add("{\"version\":\"%s\",\"uptime\":\"%" PRIu64 "\",\"rev\":\"%" PRIu64 "\",\"job\":\"%" PRIu64 "\",\"save\":\"%s\",\"storage\":\"%s\",\"cooldown\":%" PRIu64 ",",
    Version,now/1000,m.rev,m.job,results[m.result],stores[m.store],now<freeAt ? (freeAt-now+999)/1000:0);
  t.add("\"ready\":%s,\"inhibit\":%s,\"restarting\":%s,\"restartIn\":%" PRIu64 ",\"workerFault\":%s,\"ssid\":",
    boolean(m.ready),boolean(v.inhibit || !m.ready),boolean(m.deadline!=0),m.deadline>now?(m.deadline-now+999)/1000:0,boolean(m.workerFault));
  t.string(apSsid);
  t.add(",\"savedSsid\":"); t.string(cfg.ssid);
  t.add(",\"stations\":%u,\"heap\":%u,\"minHeap\":%u,\"largest\":%u,\"bootReason\":%d,\"dhtFailures\":\"%" PRIu64 "\",\"oled\":\"%s\",",
    static_cast<unsigned>(WiFi.softAPgetStationNum()),static_cast<unsigned>(ESP.getFreeHeap()),static_cast<unsigned>(ESP.getMinFreeHeap()),
    static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)),static_cast<int>(esp_reset_reason()),d.failures,
    !cfg[OledOn]?"disabled":m.oledOK?"ready":"waiting");
  t.add("\"alarms\":%u,\"outputs\":%u,\"relayMode\":%u,\"state\":[\"%s\",\"%s\",\"%s\",\"%s\"],\"warmGas\":%" PRIu64 ",\"warmPir\":%" PRIu64 ",",
    static_cast<unsigned>(v.alarms),static_cast<unsigned>(v.outputs),static_cast<unsigned>(cfg[RelayMode]),
    readings[v.state[0]],readings[v.state[1]],readings[v.state[2]],readings[v.state[3]],v.warmGas,v.warmPir);
  t.add("\"temp\":%" PRId64 ",\"hum\":%" PRId64 ",\"gas\":%" PRId64 ",\"light\":%" PRId64 ",\"pir\":%s,\"held\":%s}",v.temp,v.hum,v.gas,v.light,boolean(v.pir),boolean(v.held));
  if (t.ok) json(c,"200 OK",c.data); else json(c,"500 Internal Server Error","{}");
}

bool unsignedNumber(const char *s,uint64_t &out) {
  uint64_t value=0;
  if (!*s) return false;
  for (;*s;++s) {
    if (*s<'0' || *s>'9') return false;
    const unsigned digit=*s-'0';
    if (value>(UINT64_MAX-digit)/10) return false;
    value=value*10+digit;
  }
  out=value; return true;
}

int nibble(unsigned char c) {
  if (c>='0' && c<='9') return c-'0';
  if (c>='a' && c<='f') return c-'a'+10;
  if (c>='A' && c<='F') return c-'A'+10;
  return -1;
}

bool unescape(char *s) {
  char *out=s;
  while (*s) {
    unsigned char n=*s++;
    if (n=='+') n=' ';
    else if (n=='%') {
      if (!s[0] || !s[1]) return false;
      const int a=nibble(s[0]),b=nibble(s[1]);
      if (a<0 || b<0) return false;
      n=(a<<4)|b; s+=2;
    }
    if (n<32 || n>126) return false;
    *out++=n;
  }
  *out=0; return true;
}

bool patch(char *body,Config &cfg,uint64_t &rev) {
  uint64_t seen=0;
  for (;;) {
    char *next=strchr(body,'&');
    if (next) *next++=0;
    char *value=strchr(body,'=');
    if (!value) return false;
    *value++=0;
    if (!unescape(body) || !unescape(value)) return false;
    size_t index=0;
    while (index<KeyCount && strcmp(body,Specs[index].name)!=0) ++index;
    if (index==KeyCount) {
      if (!strcmp(body,"rev")) index=KeyCount;
      else if (!strcmp(body,"ssid")) index=KeyCount+1;
      else if (!strcmp(body,"pass")) index=KeyCount+2;
      else return false;
    }
    const uint64_t bit=UINT64_C(1)<<index;
    if (seen&bit) return false;
    seen|=bit;
    if (index==KeyCount) { if (!unsignedNumber(value,rev)) return false; }
    else if (index==KeyCount+1) {
      if (!printable(value,sizeof cfg.ssid,1)) return false;
      memset(cfg.ssid,0,sizeof cfg.ssid); strcpy(cfg.ssid,value);
    } else if (index==KeyCount+2) {
      if (*value) {
        if (!printable(value,sizeof cfg.pass,8)) return false;
        memset(cfg.pass,0,sizeof cfg.pass); strcpy(cfg.pass,value);
      }
    } else {
      const bool neg=*value=='-'; uint64_t n=0;
      if (!unsignedNumber(value+neg,n) || n>INT64_MAX) return false;
      cfg.v[index]=neg?-static_cast<int64_t>(n):static_cast<int64_t>(n);
    }
    if (!next) break;
    body=next;
  }
  const uint64_t revBit=UINT64_C(1)<<KeyCount;
  return (seen&revBit) && (seen&~revBit) && validConfig(cfg);
}

bool goodToken(const char *s) {
  if (!s || strlen(s)!=32 || !csrf[0]) return false;
  unsigned diff=0;
  for (size_t i=0;i<32;++i) diff|=static_cast<unsigned char>(s[i]^csrf[i]);
  return diff==0;
}

void saveConfig(Client &c,const Request &r) {
  if (!goodToken(r.token) || (r.origin && strcmp(r.origin,PortalUrl)!=0)) {
    json(c,"403 Forbidden",R"({"error":"Session expired or origin not permitted. Reload the page."})"); return;
  }
  if (!r.type || strcasecmp(r.type,"application/x-www-form-urlencoded")!=0) {
    json(c,"415 Unsupported Media Type",R"({"error":"Expected URL-encoded settings."})"); return;
  }
  Config cfg,changed; Meta m; snapshot(cfg,m); changed=cfg;
  uint64_t rev=0;
  if (!m.ready || m.workerFault) { json(c,"503 Service Unavailable",R"({"error":"Settings storage is not ready."})"); return; }
  if (!patch(c.request+c.split,changed,rev)) {
    json(c,"422 Unprocessable Content",R"({"error":"Invalid settings. Check bounds, alarm hysteresis, and output timing."})"); return;
  }
  const Ms now=nowMs();
  const bool different=memcmp(&cfg,&changed,sizeof cfg)!=0, wifi=!sameWifi(cfg,changed);
  uint64_t id=0; int result=0;
  portENTER_CRITICAL(&guard);
  if (meta.result==Pending || meta.resetRequested || meta.resetPending || meta.deadline) result=1;
  else if (rev!=m.rev || rev!=meta.rev) result=2;
  else if (different && now<saveAt(meta)) result=3;
  else if (different && meta.job==UINT64_MAX) result=4;
  else if (different) {
    id=++meta.job; queued=Job{changed,id,false,wifi}; meta.result=Pending; meta.haveAttempt=true; meta.lastAttempt=now;
    meta.writeTimes[meta.writeIndex]=now; meta.writeIndex=(meta.writeIndex+1)%SaveBudget;
    if (meta.writeCount<SaveBudget) ++meta.writeCount;
  }
  portEXIT_CRITICAL(&guard);
  if (result==1) json(c,"409 Conflict",R"({"error":"Another changeset or restart is in progress. Try again shortly."})");
  else if (result==2) json(c,"409 Conflict",R"({"error":"Settings changed in another session. Reload settings before saving."})");
  else if (result==3) json(c,"429 Too Many Requests",R"({"error":"Flash write budget is active. Check the save cooldown in System status before retrying."})");
  else if (result==4) json(c,"503 Service Unavailable",R"({"error":"Transaction counter exhausted."})");
  else if (!different) json(c,"200 OK",R"({"unchanged":true})");
  else {
    snprintf(c.data,sizeof c.data,"{\"job\":\"%" PRIu64 "\",\"wifi\":%s}",id,boolean(wifi));
    json(c,"202 Accepted",c.data);
  }
}

bool parseHeader(Client &c,Request &r) {
  char *line=c.request;
  char *next=strstr(line,"\r\n");
  if (!next) return false;
  *next=0; next+=2;
  r.method=line;
  r.path=strchr(line,' ');
  if (!r.path) return false;
  *r.path++=0;
  char *http=strchr(r.path,' ');
  if (!http || r.path[0]!='/') return false;
  *http++=0;
  const bool v11=!strcmp(http,"HTTP/1.1");
  if (!v11 && strcmp(http,"HTTP/1.0")) return false;
  for (const char *s=r.path;*s;++s) if (*s<=32 || *s>=127) return false;
  if (strlen(r.path)>384 || !*r.method) return false;
  for (const char *s=r.method;*s;++s) if (*s<'A' || *s>'Z') return false;
  while (next[0]!='\r' || next[1]!='\n') {
    line=next; next=strstr(line,"\r\n");
    if (!next) return false;
    *next=0; next+=2;
    char *value=strchr(line,':');
    if (!value || value==line) return false;
    *value++=0;
    for (const char *s=line;*s;++s) {
      const unsigned char u=*s;
      if (!((u>='a' && u<='z') || (u>='A' && u<='Z') || (u>='0' && u<='9') || strchr("!#$%&'*+-.^_`|~",u))) return false;
    }
    while (*value==' ' || *value=='\t') ++value;
    size_t n=strlen(value);
    while (n && (value[n-1]==' ' || value[n-1]=='\t')) value[--n]=0;
    for (const unsigned char *s=reinterpret_cast<const unsigned char*>(value);*s;++s) if ((*s<32 && *s!=9) || *s==127) return false;
    char **slot=nullptr;
    if (!strcasecmp(line,"Host")) slot=&r.host;
    else if (!strcasecmp(line,"Origin")) slot=&r.origin;
    else if (!strcasecmp(line,"X-Field-Token")) slot=&r.token;
    else if (!strcasecmp(line,"Content-Type")) slot=&r.type;
    else if (!strcasecmp(line,"Transfer-Encoding") || !strcasecmp(line,"Expect")) return false;
    else if (!strcasecmp(line,"Content-Length")) {
      uint64_t len=0;
      if (r.hasLength || !unsignedNumber(value,len) || len>MaxBody) return false;
      r.hasLength=true; r.length=static_cast<size_t>(len);
    }
    if (slot) { if (*slot || !*value) return false; *slot=value; }
  }
  return !v11 || r.host;
}

void route(Client &c,const Request &r) {
  char *query=strchr(r.path,'?'); if (query) *query=0;
  c.head=!strcmp(r.method,"HEAD");
  const bool read=c.head || !strcmp(r.method,"GET"), post=!strcmp(r.method,"POST");
  const bool local=r.host && (!strcmp(r.host,PortalHost) || !strcmp(r.host,"192.168.4.1:80"));
  if (!local && read) { redirect(c); return; }
  if (!local) { json(c,"403 Forbidden","{}"); return; }
  if (read && !r.length) {
    if (!strcmp(r.path,"/")) respond(c,"200 OK","text/html; charset=utf-8",Page,sizeof Page-1);
    else if (!strcmp(r.path,"/api/status")) getStatus(c);
    else if (!strcmp(r.path,"/api/config")) getConfig(c);
    else if (!strncmp(r.path,"/api/",5) || !strcmp(r.path,"/favicon.ico")) json(c,"404 Not Found","{}");
    else redirect(c);
  } else if (post && !strcmp(r.path,"/api/config")) {
    if (!r.hasLength || !r.length) json(c,"400 Bad Request","{}");
    else saveConfig(c,r);
  } else json(c,"405 Method Not Allowed","{}");
}

bool inspect(Client &c) {
  c.request[c.used]=0;
  if (!c.split) {
    const char *end=strstr(c.request,"\r\n\r\n");
    if (!end && memchr(c.request,0,c.used)) { json(c,"400 Bad Request","{}"); return true; }
    if (!end && c.used<MaxHeader) return false;
    const size_t split=end?static_cast<size_t>(end-c.request)+4:0;
    if (!split || split>MaxHeader) { json(c,"431 Request Header Fields Too Large","{}"); return true; }
    c.split=split;
    c.parsed=Request{};
    if (!parseHeader(c,c.parsed)) { json(c,"400 Bad Request","{}"); return true; }
    c.head=!strcmp(c.parsed.method,"HEAD");
  }
  const Request &r=c.parsed;
  if (c.used<c.split+r.length) return false;
  if (c.used!=c.split+r.length || memchr(c.request+c.split,0,r.length)) { json(c,"400 Bad Request","{}"); return true; }
  route(c,r); return true;
}

int openSocket(int type,uint16_t port,uint32_t bindAddress=0xc0a80401) {
  const int fd=socket(AF_INET,type,0);
  if (fd<0) return -1;
  const int one=1;
  sockaddr_in local{}; local.sin_family=AF_INET; local.sin_port=htons(port); local.sin_addr.s_addr=htonl(bindAddress);
  setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
  if (fcntl(fd,F_SETFL,O_NONBLOCK)<0 || bind(fd,reinterpret_cast<sockaddr*>(&local),sizeof local)<0 ||
      (type==SOCK_STREAM && listen(fd,Slots)<0)) { close(fd); return -1; }
  return fd;
}

void httpTick(Ms now) {
  for (auto &c:clients) {
    if (c.fd<0) continue;
    if (now>=c.idle || now>=c.absolute) { closeClient(c); continue; }
    if (!c.sending) {
      const size_t remaining=sizeof c.request-1-c.used;
      if (!remaining) { json(c,"413 Content Too Large","{}"); continue; }
      const int n=recv(c.fd,c.request+c.used,remaining,0);
      if (n==0 || (n<0 && !retrySocket())) { closeClient(c); continue; }
      if (n>0) { c.used+=n; c.idle=now+2500; inspect(c); }
      continue;
    }
    const bool header=c.sent<c.headerLen;
    const char *data=header?c.header+c.sent:c.body+(c.sent-c.headerLen);
    const size_t left=header?c.headerLen-c.sent:c.headerLen+c.len-c.sent;
    if (!left) { closeClient(c); continue; }
    const int n=send(c.fd,data,left<WriteChunk?left:WriteChunk,0);
    if (n>0) {
      c.sent+=n; c.idle=now+2500;
      if (c.sent==c.headerLen+c.len) closeClient(c);
    } else if (n==0 || !retrySocket()) closeClient(c);
  }
  for (auto &c:clients) if (c.fd<0) {
    const int fd=accept(tcp,nullptr,nullptr);
    if (fd<0) break;
    if (fcntl(fd,F_SETFL,O_NONBLOCK)<0) { close(fd); continue; }
    c.fd=fd; c.idle=now+2500; c.absolute=now+10000; c.used=0; c.split=0; c.sending=false; c.head=false;
  }
}

size_t dnsReply(uint8_t *p,size_t n,size_t cap) {
  if (n<17 || cap<n || (p[2]&0xf8) || p[4] || p[5]!=1 || p[6] || p[7] || p[8] || p[9]) return 0;
  size_t i=12;
  while (i<n && p[i]) {
    const size_t m=p[i++];
    if (m>63 || i+m>=n || i+m>=12+255) return 0;
    i+=m;
  }
  if (i+5>n) return 0;
  const unsigned type=(p[i+1]<<8)|p[i+2], klass=(p[i+3]<<8)|p[i+4];
  i+=5;
  const uint8_t answer[]={0xc0,0x0c,0,1,0,1,0,0,0,0,0,4,192,168,4,1};
  const bool add=klass==1 && (type==1 || type==255);
  if (add && i+sizeof answer>cap) return 0;
  p[2]=0x84|(p[2]&1); p[3]=0; p[7]=add?1:0; p[10]=p[11]=0;
  if (add) { memcpy(p+i,answer,sizeof answer); i+=sizeof answer; }
  return i;
}

void dnsTick() {
  for (unsigned i=0;i<4;++i) {
    uint8_t pkt[512]; sockaddr_in remote{}; socklen_t len=sizeof remote;
    const int n=recvfrom(dns,pkt,sizeof pkt,0,reinterpret_cast<sockaddr*>(&remote),&len);
    if (n<=0) return;
    const size_t reply=dnsReply(pkt,n,sizeof pkt);
    if (reply) sendto(dns,pkt,reply,0,reinterpret_cast<sockaddr*>(&remote),len);
  }
}

// ========================================================
// Access point startup, workload isolation, and Arduino entry
// ========================================================
void networkTask(void *) {
  const IPAddress ip(192,168,4,1), mask(255,255,255,0), lease(192,168,4,2);
  bool ap=false,bootConfigSet=false;
  Config bootConfig{};
  Ms retry=0,health=0;
  WiFi.persistent(false);
  for (;;) {
    const Ms now=nowMs();
    if (ap && now>=health) {
      health=now+10000;
      if (WiFi.getMode()!=WIFI_AP || WiFi.softAPIP()!=ip) {
        for (auto &c:clients) closeClient(c);
        if (tcp>=0) close(tcp);
        if (dns>=0) close(dns);
        tcp=dns=-1; ap=false; retry=now;
      }
    }
    if (now>=retry && (!ap || tcp<0 || dns<0)) {
      retry=now+5000;
      Config c; Meta m; snapshot(c,m);
      if (m.ready || m.workerFault) {
        if (!bootConfigSet) { bootConfig=c; bootConfigSet=true; }
        if (!ap) {
          ap=WiFi.mode(WIFI_AP) && WiFi.softAPConfig(ip,ip,mask,lease,ip) &&
             WiFi.softAP(bootConfig.ssid,bootConfig.pass,bootConfig[ApChan],0,bootConfig[ApMax],false,WIFI_AUTH_WPA2_PSK,WIFI_CIPHER_TYPE_CCMP);
          health=now+10000;
          if (ap) {
            strcpy(apSsid,bootConfig.ssid);
            if (!csrf[0]) {
              uint8_t bytes[16]; esp_fill_random(bytes,sizeof bytes);
              constexpr char hex[]="0123456789abcdef";
              for (size_t i=0;i<sizeof bytes;++i) { csrf[2*i]=hex[bytes[i]>>4]; csrf[2*i+1]=hex[bytes[i]&15]; }
            }
          } else WiFi.mode(WIFI_OFF);
        }
        if (ap) {
          if (tcp<0) tcp=openSocket(SOCK_STREAM,80);
          if (dns<0) dns=openSocket(SOCK_DGRAM,53);
        }
      } else retry=now+100;
    }
    if (dns>=0) dnsTick();
    if (tcp>=0) httpTick(now);
    yieldMs(5);
  }
}

constexpr uint32_t StoreStack=5120,ClimateStack=4096,OledStack=6144,NetStack=10240;
StaticTask_t storeTaskMem,climateTaskMem,oledTaskMem,netTaskMem;
StackType_t storeStack[StoreStack/sizeof(StackType_t)], climateStack[ClimateStack/sizeof(StackType_t)],
  oledStack[OledStack/sizeof(StackType_t)], netStack[NetStack/sizeof(StackType_t)];
bool watchdog=false;

void initialize() {
  for (const auto pin:OutPins) {
    gpio_reset_pin(pin); gpio_set_level(pin,0); gpio_set_direction(pin,GPIO_MODE_OUTPUT);
  }
  esp_log_level_set("*",ESP_LOG_NONE);
  pinMode(BootPin,INPUT_PULLUP); pinMode(PirPin,INPUT);
  initAdc();
  live=defaults();
  const auto store=xTaskCreateStaticPinnedToCore(storageTask,"field-store",StoreStack,nullptr,1,storeStack,&storeTaskMem,0);
  const auto sensor=xTaskCreateStaticPinnedToCore(climateTask,"field-dht",ClimateStack,nullptr,1,climateStack,&climateTaskMem,0);
  const auto oled=xTaskCreateStaticPinnedToCore(oledTask,"field-oled",OledStack,nullptr,1,oledStack,&oledTaskMem,0);
  const auto net=xTaskCreateStaticPinnedToCore(networkTask,"field-net",NetStack,nullptr,1,netStack,&netTaskMem,0);
  if (!store || !sensor || !oled || !net) { portENTER_CRITICAL(&guard); meta.workerFault=true; portEXIT_CRITICAL(&guard); }
  watchdog=esp_task_wdt_status(nullptr)==ESP_OK || esp_task_wdt_add(nullptr)==ESP_OK;
}
}

void setup() { Field::initialize(); }

void loop() {
  Field::control();
  if (Field::watchdog) esp_task_wdt_reset();
  Field::yieldMs(5);
}
