# ESP32-S3-DevKitC-1 — Beschreibung & Pinbelegung

> Zentrale Steuerung des GrabGeist. Gesteckt wird nur das **DevKit** (U1); alle übrigen Bauteile
> sitzen auf der Trägerplatine (SMD). Das Board trägt das Modul **ESP32-S3-WROOM-1-N16R8**
> (16 MB Flash, 8 MB PSRAM, 2,4 GHz WiFi + Bluetooth LE 5).

## Technische Daten (DevKitC-1)

- Chip: ESP32-S3 (Xtensa LX7 Dual-Core, bis 240 MHz)
- Modul: ESP32-S3-WROOM-1-**N16R8** (16 MB Flash, 8 MB PSRAM)
- Funk: WiFi 802.11 b/g/n + Bluetooth LE 5
- USB: USB-C (USB-Serial-JTAG zum Flashen + `Serial`)
- Spannungsversorgung: **5 V** (USB-C oder 5V-Pin) → Bord-LDO → **3,3 V**
- On-Board: **BOOT-Taster** (GPIO0), **RGB-LED (WS2812, GPIO48)**, RST-Taster

## Pinbelegung (Footprint, 2× Stiftleiste)

### Linke Stiftleiste (Pin 1–22, von oben)

| Pin | Name | Pin | Name |
|---|---|---|---|
| 1 | 3V3 | 12 | GPIO8 |
| 2 | 3V3 | 13 | GPIO3 (Strapping) |
| 3 | RST | 14 | GPIO46 (Strapping) |
| 4 | GPIO4 | 15 | GPIO9 |
| 5 | GPIO5 | 16 | GPIO10 |
| 6 | GPIO6 | 17 | GPIO11 |
| 7 | GPIO7 | 18 | GPIO12 |
| 8 | GPIO15 | 19 | GPIO13 |
| 9 | GPIO16 | 20 | GPIO14 |
| 10 | GPIO17 | 21 | 5V |
| 11 | GPIO18 | 22 | GND |

### Rechte Stiftleiste (Pin 44–23, von oben)

| Pin | Name | Pin | Name |
|---|---|---|---|
| 44 | GND | 33 | GPIO36 |
| 43 | GPIO43 (U0TXD) | 32 | GPIO35 |
| 42 | GPIO44 (U0RXD) | 31 | GPIO0 (BOOT) |
| 41 | GPIO1 | 30 | GPIO45 (Strapping) |
| 40 | GPIO2 | 29 | GPIO48 (RGB-LED) |
| 39 | GPIO42 (JTAG) | 28 | GPIO47 |
| 38 | GPIO41 (JTAG) | 27 | GPIO21 |
| 37 | GPIO40 (JTAG) | 26 | GPIO20 (USB) |
| 36 | GPIO39 (JTAG) | 25 | GPIO19 (USB) |
| 35 | GPIO38 | 24 | GND |
| 34 | GPIO37 | 23 | GND |

## Belegte Pins (GrabGeist)

| Footprint-Pin | GPIO | Funktion |
|---|---|---|
| 1 | 3V3 | Versorgung (+3V3-Netz) |
| 4 | GPIO4 | Servo rechts (PWM) |
| 5 | GPIO5 | Servo links (PWM) |
| 6 | GPIO6 | Akku-Teiler (ADC1_CH5) |
| 7 | GPIO7 | GPIO-Leiste |
| 12 | GPIO8 | SG90-Servo (PWM) |
| 15 | GPIO9 | GPIO-Leiste |
| 16 | GPIO10 | GPIO-Leiste |
| 21 | 5V | Versorgung (+5V-Netz) |
| 22 | GND | Masse |
| 27 | GPIO21 | CAN-D |
| 28 | GPIO47 | CAN-R |
| 35 | GPIO38 | Buzzer |
| 40 | GPIO2 | GPIO-Leiste |
| 41 | GPIO1 | Akustik-Trigger (ADC1_CH0) |
| 42 | GPIO44 (U0RXD) | UART RX |
| 43 | GPIO43 (U0TXD) | UART TX |

## Hinweise

- **Strapping-Pins (nicht verwenden):** GPIO0, GPIO3, GPIO45, GPIO46.
- **ADC1:** GPIO1–10 (bei BLE/WiFi voll nutzbar). **ADC2:** GPIO11–20 (analog eingeschränkt bei WiFi).
- **JTAG:** GPIO39–42 (nur digital, besser meiden).
- **USB-OTG:** GPIO19/20 (USB_D−/D+).
- **RGB-LED (WS2812):** GPIO48 — on-Board, per NeoPixel-Library ansteuerbar.
- **UART0:** GPIO43 (TX) / GPIO44 (RX) = `Serial0` (Arduino); `Serial` läuft über USB-C.
- **Flashen:** über USB-C (USB-Serial-JTAG); GPIO0 = BOOT-Taster.
