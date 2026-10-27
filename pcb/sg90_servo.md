# Zusatz-Servo SG90 (5-V-Mikroservo) — GrabGeist (PCB-Modul)

> Zweck: zusätzlicher Ausgang für einen **SG90-Mikroservo** (9 g, 4,8–6 V). **Achtung:** der SG90
> läuft auf **5 V**, nicht auf der LiPo-Servo-Schiene (6,0–8,4 V) — er wird deshalb am **+5V-Buck**
> betrieben, **nicht an `VCC`**.
>
> **Stand:** ins Gesamt-Schema integriert (EasyEDA). RefDes konsolidiert (siehe `power_design.md`).

## 1. Anschluss

| Pin | Funktion (Farbkodierung) | Netz |
|---|---|---|
| CN_SG90.1 | GND (braun/schwarz) | GND |
| CN_SG90.2 | +5 V (rot) | **+5V** (Buck — nicht VCC!) |
| CN_SG90.3 | Signal (orange/gelb) | GPIO8 (PWM) |

## 2. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | Funktion |
|---|---|---|---|
| CN_SG90 | 3-Pol-Stiftleiste/JST-XH 2,54 mm | THT | SG90-Anschluss (GND/+5V/Signal) |
| C14 | 100 µF / 16 V (optional) | SMD-Elko Ø6,3 | Stützkondensator nah am Stecker |

## 3. Verdrahtung / Netzliste

| Netz | Pins |
|---|---|
| GPIO8 | U1.8 (GPIO8), CN_SG90.3 |
| +5V | CN_SG90.2, C14.1 (zusätzlich zum bestehenden +5V-Netz) |
| GND | CN_SG90.1, C14.2 (zusätzlich zum bestehenden GND-Netz) |

## 4. Pinbelegung / Firmware

| Funktion | ESP32-S3-Pin | Netz | Konstante (später) |
|---|---|---|---|
| SG90-Servo (PWM) | **GPIO8** | GPIO8 | neu |

- PWM wie die großen Servos: **50 Hz**, **14 Bit** LEDC, Pulsbreite **500–2500 µs**.
- 3,3-V-Signalpegel reicht für den SG90.
- GPIO8 ist **kein Strapping-Pin** und frei (kein Konflikt mit GPIO1/4/5/6/15/16).

## 5. Hinweise

- **Wichtig: +5V statt VCC.** Der SG90 verträgt max. ~6 V; die LiPo-Schiene (6,0–8,4 V) würde ihn zerstören.
- Strom: SG90 typ. ~100–250 mA, Stall ~650 mA. Der Buck (AP63205, 2 A) versorgt DevKit + SG90 zusammen problemlos.
- Der optionale 100-µF-Elko (C14) direkt am Stecker puffert die Stromspitzen des Servos.
- Falls später mehrere SG90 geplant sind, den 5V-Stromhaushalt prüfen (Buck = 2 A).
