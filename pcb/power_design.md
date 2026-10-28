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
        │  PTC · P-MOSFET-Verpolungsschutz (Q1)
        ▼
  VCC ──┬──► C10 (470 µF Bulk, Stütze)
        ├────────────────────────► Servo L (V+)  ── Signal ◄── GPIO4   (C11 100 µF)
        ├────────────────────────► Servo R (V+)  ── Signal ◄── GPIO5   (C12 100 µF)
        ├──► 5V-Buck U2 (AP63205) ──► 5V-Pin DevKitC-1 (Bord-LDO → 3V3)
        └──► Teiler 100k/47k ───► GPIO6 (ADC1_CH5)
  GND  (gemeinsame Masse: LiPo− · Servo-GND · DevKit-GND)
```

## 4. Spannungs-/Strombudget

| Schiene | Spannung | typ. Strom (gemessen) | Auslegung |
|---|---|---|---|
| Servo-Rail (direkt am LiPo) | 6,0–8,4 V | < 1 A gesamt | 3 A Peak (Leiterbahn/Sicherung) |
| 5 V (Buck → DevKit + SG90) | 5,0 V | ~0,3–0,5 A + SG90 ~0,25 A (Stall ~0,65 A) | ≥ 1 A (AP63205 = 2 A) |
| 3,3 V (Bord-LDO des DevKit) | 3,3 V | intern | intern |

## 5. Bauteilliste (BOM, alles SMD außer Sockel)

| Ref | Bauteil | Wert/Parameter | Gehäuse | Bemerkung |
|---|---|---|---|---|
| CN-VBat | Batterie-Anschluss | JST-XH 2-pol | SMD | zur 18650-Halterung (BMS am Pack) |
| Q1 | P-MOSFET (Verpolungsschutz) | AO3401A | SOT-23 | siehe §5.2 (Drain=BAT+, Source=VCC, Gate→GND direkt) |
| F1 | Sicherung | PTC 3 A | 1812 | rücksetzbar, Kurzschlussschutz |
| C10 | Bulk-Elko (Eingang) | 470 µF / 16 V | SMD-Elko Ø8 | Stützkapazität für Servo-Spitzen |
| CN-SL, CN-SR | Servo-Anschlüsse | 3-pol 2,54 mm | SMD | JR/Futaba-kompatibel (GND/V+/Signal) |
| C11, C12 | Stütz-Elko je Servo | 100 µF / 16 V | SMD-Elko Ø6,3 | nah am Stecker |
| U2 | 5V-Buck | AP63205 (fest 5 V, 2 A, 1,1 MHz) | TSOT26 (SOT-23-6) | alternativ MP2315/TPS563201 |
| L1 | Induktivität | 10 µH, Isat ≥ 2 A | SMD | laut Buck-Datenblatt |
| C1 | CIN | 10 µF / 25 V X7R | 0805 | nah an VIN |
| C4 | Bootstrap | 0,1 µF / 16 V X7R | 0603 | zwischen BST und SW |
| C2, C3 | COUT | 2× 22 µF / 16 V X7R | 0805 | low ESR |
| R2 | Teiler oben | 100 k (1 %) | 0603 | VCC → ADC |
| R3 | Teiler unten | 47 k (1 %) | 0603 | ADC → GND |
| C5 | ADC-Filter | 100 nF | 0603 | am ADC-Knoten |
| U1 | ESP32-S3 DevKitC-1 (N16R8) | gesteckt | THT-Female-Header | die einzige „gesteckte" Ausnahme |

> Teilenummern (LCSC `C…`) im EasyEDA-Pro-Katalog auswählen.

### 5.1 AP63205 — Pinbelegung & Beschaltung

**Gehäuse:** TSOT26 (SOT-23-6), Top View · **Frequenz:** 1,1 MHz · **Ausgang:** fest 5,0 V.

| Pin | Name | Funktion |
|---|---|---|
| 1 | FB | Feedback |
| 2 | EN | Enable |
| 3 | VIN | Eingangsspannung |
| 4 | GND | Masse |
| 5 | SW | Schaltknoten (zur Induktivität) |
| 6 | BST | Bootstrap |

```
         FB        EN       VIN
        [1]       [2]       [3]
       ┌────────────────────────┐
       │                        │
       └────────────────────────┘
        [6]       [5]       [4]
        BST        SW       GND
```

**Beschaltung (typische Anwendung, fest 5 V):**

```
VCC ────┬── CIN (10 µF + 0,1 µF) ─── GND
        ├── VIN (3)
EN (2) ─┤   (an VIN für „immer an", optional UVLO-Teiler VIN→EN→GND)
        │
BST (6)─┴── CBST (0,1 µF) ── SW (5)
SW (5) ───── L (10 µH) ──┬── VOUT (5 V)
                         ├── COUT (2× 22 µF) ── GND
                         └── FB (1)   ← direkt an VOUT
GND (4) ─── GND
```

- **VIN (3):** an VCC (6,0–8,4 V). `CIN = 10 µF (X7R)` + 0,1 µF direkt am Pin.
- **GND (4):** gemeinsame Masse (GND-Fläche).
- **EN (2):** HIGH = ein — direkt an VIN („immer an", kein Pull-up nötig); optional UVLO-Teiler.
- **BST (6):** Bootstrap-Kondensator `CBST = 0,1 µF (X7R)` zwischen **BST und SW**.
- **SW (5):** Schaltknoten → `L = 10 µH` → Ausgang (bei 1,1 MHz sind 6,8–10 µH üblich).
- **FB (1):** bei festem 5-V-Ausgang **direkt an VOUT** — **kein** externer Teiler.
- **Ausgang:** `COUT = 2× 22 µF (X7R)` gegen GND.

> Pin-Nummerierung und Beschaltungsbild im EasyEDA-Pro-Symbol (LCSC-Bauteil AP63205WU-7)
> und dem verlinkten Datenblatt gegenprüfen.

### 5.2 Verpolungsschutz (P-MOSFET) & Sicherung

**Bauteile:** Q1 = AO3401A (P-Kanal, SOT-23), F1 = PTC 3 A. Gate hängt **direkt an GND** (kein Gate-Widerstand).

**AO3401A Pinbelegung (SOT-23, Top View):**

```
        1                2
      Gate             Source
      ┌───────────────────────┐
      │        AO3401A        │
      └───────────────────────┘
                    3
                  Drain
```

**Verdrahtung:**

```
Batterie+ (CN-VBat.2) ──► F1 (PTC 3 A) ──► Q1.Drain (Pin 3)
                                            Q1.Source (Pin 2) ──► VCC (geschützt)
                                            Q1.Gate (Pin 1) ──► GND (direkt, kein Widerstand)
Batterie− (CN-VBat.1) ─────────────────────────────────────────────────────► GND
```

- **Q1 Pin 3 (Drain)** → über F1 an Batterie-Plus (Drain an den Akku, Source an die Last).
- **Q1 Pin 2 (Source)** → `VCC` (geschützte Schiene: C10/Bulk, Buck U2, Servo V+).
- **Q1 Pin 1 (Gate)** → direkt nach GND (kein Gate-Widerstand nötig).
- **F1 (PTC 3 A)** in die Plus-Leitung, direkt nach dem Batteriestecker.
- Bulk-Elko (C10) auf die **geschützte `VCC`-Seite** (nach Q1).

**Funktion:**
- Richtige Polung: Body-Diode (Anode = Drain → Kathode = Source) hebt `VCC` kurz an; danach Vgs = −U (Gate = 0 V) → P-Kanal schaltet voll durch (Rds(on) ≈ 50 mΩ), Strom Drain → Source.
- Falsche Polung: Kanal sperrt (Vgs ≥ 0); die Body-Diode (Anode = Drain, Kathode = Source) liegt in Sperrrichtung → kein Strom, Last bleibt geschützt.

**Hinweise:**
- Zener zur Vgs-Klemme ist bei 2S (≤ 8,4 V) **nicht** nötig.
- Einfachste Alternative: Schottky SS34 in Serie (~0,4 V Abfall) — verliert aber Spannung.

### 5.3 LCSC-Suchbegriffe (Bauteilbeschaffung)

| Bauteil | Suchbegriff bei LCSC | Filter / Spezifikation |
|---|---|---|
| 5V-Buck (U2) | `AP63205WU-7` | Diodes, TSOT-23-6, fest 5 V, 2 A |
| P-MOSFET (Q1) | `AO3401A` | SOT-23, P-Kanal, Vds −30 V, Id −4 A |
| PTC (F1) | `1812L300/24GR` (LUTE, LCSC C20627123) | 3 A haltend / 5 A Trip / 24 V / 1812 |
| Induktivität (L1) | `10uH power inductor 2A` | 10 µH, Isat ≥ 2 A, geschirmt, 7,3×7,3 |
| CIN (C1) | `10uF 0805 25V` | X5R/X7R, 0805 |
| COUT (C2, C3) | `22uF 0805 16V` | X5R/X7R, 0805 |
| Keramik 100nF (C4/C5/C6/C8) | `100nF 0603 16V` | X7R, 0603 |
| Bulk-Elko (C10) | `470uF 16V SMD electrolytic` | ≥ 16 V (besser 25 V), SMD |
| Servo-Elko (C11, C12) | `100uF 16V SMD electrolytic` | ≥ 16 V, SMD |
| Widerstand 100k (R2) | `100k 0603 1%` | 0603, 1 % |
| Widerstand 47k (R3) | `47k 0603 1%` | 0603, 1 % |
| Batterie-Stecker (CN-VBat) | `XH 2P SMD` (z. B. `S2B-XH-SM4-TB`) | 2,54 mm, 2-pol |
| Servo-Stecker (CN-SL/CN-SR) | `2.54 3P SMD` (oder `XH 3P`) | 2,54 mm, 3-pol |

> Bei LCSC die MPN/den Suchbegriff eingeben; die `C…`-Nummer erscheint im Treffer.
> Für Keramik-/Widerstände Marken wie Samsung/Murata/TDK/Sunlord/UniOhm wählen.

### 5.4 Spannungsteiler (ADC-Spannungsmessung)

**Bauteile:** R2 = 100 kΩ (oben), R3 = 47 kΩ (unten), C5 = 100 nF (Filter).

**Verdrahtung:**

```
VCC (geschützt) ──► R2 (100 k) ──┬──► GPIO6 (ADC1_CH5)
                                         │
                                         R3 (47 k)
                                         │
                                         GND
                                         │
                                         C5 (100 nF)   (parallel zu R3, nah am GPIO)
                                         │
                                         GND
```

- **R2 (100 k):** VCC → Mittenknoten.
- **R3 (47 k):** Mittenknoten → GND.
- **GPIO6 (ADC1_CH5):** an den Mittenknoten.
- **C5 (100 nF):** Mittenknoten → GND (Rauschfilter + Puffer für den ADC-Sample).

**Teilerverhältnis & Spannungen:** `V_ADC = V_BAT × 47 / 147 = V_BAT × 0,32`

| Batterie | V_ADC | Bedeutung |
|---|---|---|
| 8,4 V (voll) | 2,69 V | oberes Ende (sicher unter 3,1 V) |
| 6,8 V | 2,17 V | Hysterese „wieder frei" |
| 6,4 V | 2,05 V | **Cutoff** (~3,2 V/Zelle) |
| 6,0 V | 1,92 V | hartes Minimum (~3,0 V/Zelle) |

**Hinweise:**
- Abgriff auf der **geschützten/geschalteten `VCC`** (nach PTC + Q1, hinter dem Hauptschalter), damit im „Aus"-Zustand nichts entlädt (Ruhestrom ~57 µA).
- Rückrechnung: `V_BAT = V_ADC × 3,128`.
- Firmware (später): `analogReadMilliVolts(GPIO6)` (11 dB) → `× 3,128 / 1000`. Cutoff ~6,4 V, Hysterese ~6,8 V.
- Einmal gegen Multimeter kalibrieren (1 %-Widerstände + ADC-Offset).

## 6. Verbindungen / Pinbelegung

| Funktion | ESP32-S3-Pin | Konstante in `../src/main.cpp` |
|---|---|---|
| Linker Servo (PWM) | **GPIO5** | `PIN_SERVO_LEFT` |
| Rechter Servo (PWM) | **GPIO4** | `PIN_SERVO_RIGHT` |
| Batteriespannung (ADC) | **GPIO6** (ADC1_CH5) | neu (Firmware-Patch, später) |
| Akustik-Trigger (ADC) | **GPIO1** (ADC1_CH0) | neu (siehe `akustik_trigger.md`) |
| CAN TX (D) | **GPIO21** | neu (siehe `can_bus.md`) |
| CAN RX (R) | **GPIO47** | neu (siehe `can_bus.md`) |
| SG90-Servo (PWM) | **GPIO8** | neu (siehe `sg90_servo.md`) |
| Buzzer (PWM) | **GPIO38** | neu (siehe `buzzer.md`) |
| GPIO-Leiste (4× frei, ADC1) | **GPIO2 / GPIO7 / GPIO9 / GPIO10** | neu (frei verfügbar) |
| UART TX | **GPIO43** (U0TXD) | neu (Debug-Header `CN-UART`) |
| UART RX | **GPIO44** (U0RXD) | neu (Debug-Header `CN-UART`) |

- PWM: **50 Hz**, **14 Bit** (LEDC), Pulsbreite 500–2500 µs (`SERVO_RES_BITS = 14`).
- 5 V vom Buck auf den **5V-Pin** (nicht 3V3). Ein gemeinsamer Ground für alles.
- **GPIO1 (ADC1_CH0)** = Akustik-Trigger (siehe `akustik_trigger.md`).
- Im konsolidierten Schema: DevKit = **U1** (Footprint-Pin ≠ GPIO-Nummer). Belegte Pins siehe `esp32_devkit.md`.

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
  Mittelabgriff (B−/BM/B+) freilegen → führt zum **2S-Protection-Board am Pack** (BMS sitzt am Akku, nicht am Print).
- **Matching (wichtig):** 4 Zellen mit ähnlicher Kapazität (±100 mAh) und Innenwiderstand
  wählen (MC5000 misst beides). Sonst driftet der Pack, eine Zelle läuft zuerst leer.
- **Laden:** Zellen **einzeln** entnehmen (Halterung, nicht fest verlötet) und im MC5000 laden;
  immer 4× frisch geladene Zellen (4,2 V) einsetzen.
- **Entladeschutz:**
  - Firmware-Cutoff (Kap. 7) — primär,
  - PTC-Sicherung (Kurzschluss),
  - **2S-Protection-Board am Pack** (Entladeseite: Übertiefentladung/Kurzschluss/Überstrom) als Hardware-Netz,
  - Hauptschalter (Kipp-/Wippschalter, siehe §10) — oder Stecker ziehen, damit im Ruhezustand nichts langsam entlädt.

## 9. Layout-/Routing-Hinweise

- **Eine gemeinsame GND-Fläche** (LiPo− · Servo-GND · DevKit-GND, sternförmig/Fläche).
- Servo-Leiterbahnen **kurz + breit** (3-A-Headroom).
- Stützkondensatoren **nah an der Last**: C10 am Eingang, C11/C12 direkt am Servo-Stecker.
- Buck: C1 (CIN) **nah an VIN**, kurze Schaltknoten, GND-Referenz zum Buck.
- Verpolungsschutz + Sicherung **vor** allen anderen Baugruppen.
- **Nie USB und Buck gleichzeitig** betreiben (Rückfluss); zum Flashen Buck trennen.
- 3,3-V-Signal reicht meist für die Servos (sonst Levelshifter, siehe `../verkabelung.md` §7).

## 10. Offene Punkte

- [x] 2S-BMS: **am Akku-Pack** (2S-Protection-Board, Entladeseite) — nicht am Print.
- [x] Batterie-Stecker: **JST-XH 2-pol** (lagernd) — nicht XT30.
- [ ] Servo-Header: 3-pol 2,54 mm (im Schema CN-SL/CN-SR); Bestückung nach vorhandenen Steckern.
- [x] Buck-IC: AP63205 (fest 5 V) — festgelegt (im Schema U2).
- [x] Hauptschalter: Kipp-/Wippschalter **SPST, ≥ 3 A @ 12 V DC (besser 5 A)**, in der Plus-Leitung vor F1/Q1 — oder Stecker ziehen (JST-XH).
- [x] Verpolungsschutz: P-MOSFET AO3401A, Gate direkt an GND (kein Gate-Widerstand); EN direkt an VIN.

## 11. Verweise

- `../verkabelung.md` — Ist-Aufbau, Servo-Daten (Miuzei MS62-2-D-B), LM2596-Prototyp
- `../SystemSpecification.md` — autoritative Spec (Kinematik, Servo-PWM)
- `../src/main.cpp` — `PIN_SERVO_LEFT`/`PIN_SERVO_RIGHT`, PWM-Parameter

