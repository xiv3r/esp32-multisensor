# esp32-multisensor
ESP32 IoT Environmental Monitoring System with multiple sensors.

# Wiring Connections
>Left Side (Inputs):

- DHT22:
```
Red wire -> 3.3V
Black wire -> GND
White wire (Signal) -> GPIO 4
```
- LDR:
```
Red wire -> 3.3V
Black wire -> GND
Yellow wire (AO) -> GPIO 34
```
- PIR:
```
Red wire -> VIN (5V)
Black wire -> GND
Green wire (OUT) -> GPIO 27
```
- MQ-2:
```
Red wire -> VIN (5V)
Black wire -> GND
Yellow wire (AO) -> GPIO 35
```

> Right Side (Outputs):

- OLED Display:
```
Red wire -> GND
Black wire -> 3.3V
Blue wire (SCL) -> GPIO 22
Yellow wire (SDA) -> GPIO 21
```
- Buzzer:
```
Red wire -> GND
Black wire -> GPIO 26 
```
- Relay Module:
```
Red wire -> VIN (5V)
Black wire -> GND
Green wire (IN) -> GPIO 25
```
- LED Indicator:
```
Red wire (Anode) -> GPIO 33
Black wire (Cathode) -> GND
```
