# PCB-Stromversorgung — GrabGeist (2S2P-LiPo, SMD)

> Planungsstand der **Trägerplatine**. Ziel: kompaktes SMD-Board; gesteckt wird nur das
> **ESP32-S3 DevKitC-1 (N16R8)**. Die Firmware wird erst nach Erhalt der Platine angepasst
> (Low-Voltage-Cutoff). Gültig für die Servos **Miuzei MS62-2-D-B** (25 kg, 270°, 4,8–8,4 V,
> 500–2500 µs).

## 1. Festgelegte Entscheidungen

| Entscheidung | Wahl |
|---|---|
| Akku | **2S2P aus 4× 18650** (vorhandene Zellen); Laden **einzeln** im MC5000 |
| Servos | **direkt am LiPo** (6,0–8,4 V) — kein Wandler in der Leistungsstrecke |
| Logik | **diskreter 5-V-Buck (SMD)** → 5V-Pin des DevKit (Bord-LDO erzeugt 3,3 V) |
| Bestückung | alles **SMD**; nur das DevKit wird gesteckt (THT-Female-Header) |
| Messung | Spannungsteiler → ADC, Cutoff ~6,4 V (Firmware später) |

## 2. Gemessene Kenndaten (Ist-Stand)

- Gesamter Aufbau zieht **< 1 A** (Labornetzteil, CC-Limit 3 A wurde nie ausgelöst).
- Servo-Schiene daher leicht ausgelegt: **3 A Peak** als Headroom (Klemm-/Anlaufstrom).
- Prototyp läuft bereits mit 2S-LiPo + 5V-Buck (LM2596) — siehe `../verkabelung.md` §11.

## 3. Blockschaltbild

```
2S2P-LiPo (4× 18650)   voll 8,4 V · leer ~6,0–6,6 V
        │  Schalter · PTC · Verpolungsschutz
        ▼
  V_BAT ─┬────────────────────────► Servo L (V+)  ── Signal ◄── GPIO4
         ├────────────────────────► Servo R (V+)  ── Signal ◄── GPIO5
         ├──► 5V-Buck (AP63205) ──► 5V-Pin DevKitC-1 (Bord-LDO → 3V3)
         └──► Teiler 100k/47k ───► GPIO6 (ADC1_CH5)
  GND  (gemeinsame Masse: LiPo− · Servo-GND · DevKit-GND)
```

## 4. Spannungs-/Strombudget

| Schiene | Spannung | typ. Strom (gemessen) | Auslegung |
|---|---|---|---|
| Servo-Rail (direkt am LiPo) | 6,0–8,4 V | < 1 A gesamt | 3 A Peak (Leiterbahn/Sicherung) |
| 5 V (Buck → DevKit) | 5,0 V | ~0,3–0,5 A (BLE-Spitzen) | ≥ 1 A (AP63205 = 2 A) |
| 3,3 V (Bord-LDO des DevKit) | 3,3 V | intern | intern |

## 5. Bauteilliste (BOM, alles SMD außer Sockel)

| Ref | Bauteil | Wert/Parameter | Gehäuse | Bemerkung |
|---|---|---|---|---|
| J1 | Batterie-Anschluss | JST-XH 2-pol | SMD | zur 18650-Halterung (oder XT30) |
| Q1 | P-MOSFET (Verpolungsschutz) | AO3401A | SOT-23 | Source=BAT+, Drain=V_BAT, Gate→GND (100k); alternativ Schottky SS34 |
| F1 | Sicherung | PTC 3 A | 1812 | rücksetzbar, Kurzschlussschutz |
| C1 | Bulk-Elko (Eingang) | 470 µF / 16 V | SMD-Elko | Stützkapazität für Servo-Spitzen |
| C2 | Keramik | 100 nF / 16 V | 0603 | parallel zu C1 |
| J2, J3 | Servo-Anschlüsse | 3-pol 2,54 mm | SMD | JR/Futaba-kompatibel (GND/V+/Signal) |
| C3, C4 | Stütz-Elko je Servo | 100 µF / 16 V | SMD-Elko | nah am Stecker |
| U1 | 5V-Buck | AP63205 (fest 5 V, 2 A) | SOT-563 | alternativ MP2315/TPS563201 |
| L1 | Induktivität | 10 µH, Isat ≥ 2 A | SMD | laut Buck-Datenblatt |
| C5 | CIN | 10 µF / 25 V X7R | 0805 | nah an VIN |
| C6, C7 | COUT | 2× 22 µF / 16 V X7R | 0805 | low ESR |
| R1 | EN-Pull-up | 100 k | 0603 | VIN → EN |
| R2 | Teiler oben | 100 k (1 %) | 0603 | V_BAT → ADC |
| R3 | Teiler unten | 47 k (1 %) | 0603 | ADC → GND |
| C8 | ADC-Filter | 100 nF | 0603 | am ADC-Knoten |
| J4, J5 | DevKit-Sockel | 2× Female-Header 2,54 mm | THT | die einzige „gesteckte" Ausnahme |

> Teilenummern (LCSC `C…`) im EasyEDA-Pro-Katalog auswählen.

## 6. Verbindungen / Pinbelegung

| Funktion | ESP32-S3-Pin | Konstante in `../src/main.cpp` |
|---|---|---|
| Linker Servo (PWM) | **GPIO4** | `PIN_SERVO_LEFT` |
| Rechter Servo (PWM) | **GPIO5** | `PIN_SERVO_RIGHT` |
| Batteriespannung (ADC) | **GPIO6** (ADC1_CH5) | neu (Firmware-Patch, später) |

- PWM: **50 Hz**, **14 Bit** (LEDC), Pulsbreite 500–2500 µs (`SERVO_RES_BITS = 14`).
- 5 V vom Buck auf den **5V-Pin** (nicht 3V3). Ein gemeinsamer Ground für alles.

## 7. Spannungsüberwachung (ADC-Teiler)

Teiler **100k/47k** → ADC-Spannung:

| Batterie | ADC-Spannung | Bedeutung |
|---|---|---|
| 8,4 V (voll) | 2,69 V | oberes Ende |
| 6,4 V | 2,05 V | **Cutoff-Schwelle** (~3,2 V/Zelle) |
| 6,0 V | 1,92 V | hartes Minimum (~3,0 V/Zelle) |

- Cutoff ~**6,4 V** mit Hysterese (wieder frei ab ~6,8 V): Servos stoppen + Meldung über BLE.
- Messung über `analogRead`/`analogReadMilliVolts` auf GPIO6; Kalibrierung gegen Multimeter.
- **Umsetzung erst, wenn die Platine da ist** (per Absprache).

## 8. 18650-Pack: Aufbau & Schutz

- **Verdrahtung 2S2P:** je zwei Zellen parallel (P-Gruppe), dann die zwei P-Gruppen in Serie.
  Mittelabgriff (B−/BM/B+) für ein optionales 2S-BMS freilegen.
- **Matching (wichtig):** 4 Zellen mit ähnlicher Kapazität (±100 mAh) und Innenwiderstand
  wählen (MC5000 misst beides). Sonst driftet der Pack, eine Zelle läuft zuerst leer.
- **Laden:** Zellen **einzeln** entnehmen (Halterung, nicht fest verlötet) und im MC5000 laden;
  immer 4× frisch geladene Zellen (4,2 V) einsetzen.
- **Entladeschutz:**
  - Firmware-Cutoff (Kap. 7) — primär,
  - PTC-Sicherung (Kurzschluss),
  - optional 2S-BMS nur auf der Entladeseite (Übertiefentladung/Kurzschluss) als Hardware-Netz,
  - Hauptschalter (oder Stecker ziehen), damit im Ruhezustand nichts langsam entlädt.

## 9. Layout-/Routing-Hinweise

- **Eine gemeinsame GND-Fläche** (LiPo− · Servo-GND · DevKit-GND, sternförmig/Fläche).
- Servo-Leiterbahnen **kurz + breit** (3-A-Headroom).
- Stützkondensatoren **nah an der Last**: C1 am Eingang, C3/C4 direkt am Servo-Stecker.
- Buck: C5 **nah an VIN**, kurze Schaltknoten, GND-Referenz zum Buck.
- Verpolungsschutz + Sicherung **vor** allen anderen Baugruppen.
- **Nie USB und Buck gleichzeitig** betreiben (Rückfluss); zum Flashen Buck trennen.
- 3,3-V-Signal reicht meist für die Servos (sonst Levelshifter, siehe `../verkabelung.md` §7).

## 10. Offene Punkte

- [ ] 2S-BMS (Entladeseite) ja/nein?
- [ ] Steckertypen final: Batterie JST-XH vs. XT30; Servo-Header Bauform.
- [ ] Buck-IC final (Vorschlag AP63205).
- [ ] Hauptschalter-Bauform.

## 11. Verweise

- `../verkabelung.md` — Ist-Aufbau, Servo-Daten (Miuzei MS62-2-D-B), LM2596-Prototyp
- `../SystemSpecification.md` — autoritative Spec (Kinematik, Servo-PWM)
- `../src/main.cpp` — `PIN_SERVO_LEFT`/`PIN_SERVO_RIGHT`, PWM-Parameter

