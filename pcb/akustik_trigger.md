# Akustischer Schussknall-Trigger — GrabGeist (PCB-Modul)

> Zweck: analoge Aufbereitung eines akustischen Impulses (Schussknall / Kugelschlag in
> 10–25 m Distanz) zur Auslösung eines Ziels über den ADC des ESP32-S3.
> Versorgung **+3,3 V** (vom DevKit-LDO), Ruhepegel am Ausgang **~0 V**
> (nur die positiven Halbwellen werden verstärkt = Single-Supply-Gleichrichtung).
>
> **Stand:** ins Gesamt-Schema integriert. Die RefDes hier entsprechen der **konsolidierten
> Netzliste** des Trägerboards (siehe `power_design.md`) — sie weichen von der ersten Planung ab.

## 1. Signalweg

1. Elektret-Mikrofon mit DC-Bias (R4 = 2,2 kΩ nach +3V3, über CN-Jumper).
2. RC-Hochpass 1. Ordnung (C7/R5, fc ≈ 1,59 kHz) — filtert Wind, Trittschall, Motorenbrumm.
3. Nicht-invertierender Verstärker (U3A): **AC-Gain G = 1 + R7/R6 = 101 ≈ 40 dB, DC-Gain = 1**
   (durch C8 in Reihe zu R6). R6 + C8 bilden einen **2. Hochpass** (fc ≈ 1,06 kHz).
4. Spitzenwert-/Hüllkurvendetektor (D1, C9, R1, τ = R1·C9 = 1 ms) — dehnt µs-Impulse auf 1–3 ms,
   damit einfaches ADC-Polling (2–4 kHz) ohne Hardware-Interrupt genügt.

## 2. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | Funktion |
|---|---|---|---|
| U3 | **MCP602-I/SN** (LCSC C29605; alternativ MCP6002) | SOIC-8 | Dual Rail-to-Rail-I/O-OpAmp, 2,8 MHz GBW, 3,3-V-tauglich; **kein** LM358 |
| MIC1 | Elektret-Kapsel **GMI9745P-2C44DB** (INGHAi, LCSC C530035) | THT 9,7 mm | Variante A: Kapsel direkt auf der Platine |
| CN-Elektret | 2-Pin-Header (JST-XH) | THT 2,54 mm | Variante B: externe Kapsel über verdrilltes Kabel |
| CN-Jumper | Lötbrücke / Jumper (optional) | SMD | in Serie zu R4 (trennt Bias, falls Piezo statt Elektret) |
| D1 | **BAT54** (SOD-523F, LCSC C283259) — alternativ BAT43/BAT43W | SOD-523F / DO-35 / SOD-123 | Kleinsignal-Schottky (40 V, 300 mA), Spitzenwertgleichrichter |
| R4 | 2,2 kΩ | 0805 | Mikrofon-Bias nach +3V3 |
| R5 | 10 kΩ | 0805 | Hochpass + DC-Arbeitspunkt (0 V) für INA+ |
| R6 | 1,5 kΩ | 0805 | Gain-Fußpunkt (AC) — in Serie mit C8 nach GND |
| R7 | 150 kΩ | 0805 | Gegenkopplung (G = 1 + R7/R6 = 101) |
| R1 | 100 kΩ | 0805 | Entladewiderstand der Hüllkurve (τ = 1 ms) |
| C7 | 10 nF (X7R/C0G) | 0805 | DC-Entkopplung + Hochpass (fc = 1,59 kHz) |
| C9 | 10 nF (X7R) | 0805 | Haltekondensator (τ = R1·C9 = 1 ms) |
| C6 | 100 nF (X7R) | 0805 | Decoupling für U3 (nah an Pin 8) |
| C8 | 100 nF (X7R) | 0805 | in Reihe zu R6 → DC-Gain = 1 + 2. Hochpass (fc ≈ 1,06 kHz) |

> **Mikrofon-Varianten:** MIC1 und CN-Elektret liegen parallel auf `MIC_RAW`. Es wird **nur eine**
> Variante bestückt: MIC1 = Kapsel auf der Platine, CN-Elektret = externe Kapsel über Kabel.

## 3. MCP6002 / MCP602 — Pinbelegung (SOIC-8)

| Pin | Name | Funktion |
|---|---|---|
| 1 | OUTA | Ausgang Kanal A |
| 2 | INA− | invertierender Eingang A |
| 3 | INA+ | nicht-invertierender Eingang A |
| 4 | VSS | Masse (GND) |
| 5 | INB+ | nicht-inv. Eingang B (→ GND, stillgelegt) |
| 6 | INB− | inv. Eingang B (→ OUTB, Follower) |
| 7 | OUTB | Ausgang Kanal B |
| 8 | VDD | Versorgung +3,3 V |

## 4. Netzliste / Verdrahtung

| Netz | Pins |
|---|---|
| +3V3 | U3.8 (VDD), C6.1, CN-Jumper.1 |
| GND | U3.4 (VSS), U3.5 (INB+), C6.2, C8.1, C9.2, CN-Elektret.2, MIC1.2, R5.1, R1.2 |
| MIC_RAW | CN-Elektret.1, MIC1.1, R4.2, C7.1 |
| AMP_IN_P | C7.2, R5.2, U3.3 (INA+) |
| AMP_IN_N | U3.2 (INA−), R6.2, R7.2 |
| GAIN_RC | R6.1, C8.2 — R6 nicht direkt an GND, sondern über C8 |
| AMP_OUT | U3.1 (OUTA), R7.1, D1.2 (Anode) |
| ADC_TRIG_OUT | D1.1 (Kathode), C9.1, R1.1, U1.7 (GPIO7) |
| OPAMP_B_PARK | U3.6 (INB−), U3.7 (OUTB) — ungenutzten Kanal als Follower terminieren |

## 5. Layout-Regeln

- C6 so nah wie möglich zwischen U3 Pin 8 (VDD) und GND.
- AMP_IN_N / AMP_IN_P (U3 Pin 2/3) extrem kurz führen; **nicht** parallel zu PWM-/Servo-/Buck-Leitungen.
- Analoge Sektion über geschlossener GND-Fläche, abseits der Servo-Rückströme.
- **D1-Polarität:** Anode an `AMP_OUT` (D1.2), Kathode an `ADC_TRIG_OUT` (D1.1) — Footprint-Pin 1 = Kathode prüfen.
- Mikrofon-Varianten: **MIC1** direkt bestücken (Kapsel auf der Platine) **oder** Kapsel über
  **CN-Elektret** mit kurzem verdrilltem Kabel anschließen und in Schaumstoff lagern (Körper-/Windentkopplung).

## 6. Firmware-Schnittstelle (später)

- Signal: 0 … 3,1 V an ADC_TRIG_OUT (GPIO7, ADC1_CH6, 11-dB-Dämpfung).
- Polling `analogRead()` alle 250–500 µs im Zustand ARMED.
- Lockout während der Bewegung (RUNNING/HOMING); nach Stopp 300–500 ms Settling, dann wieder ARMEN.

## 7. Integration ins GrabGeist-PCB

- **+3V3** vom 3V3-Pin des DevKit (Bord-LDO); Zusatzstrom ~0,6 mA (OpAmp ~0,1 mA + Mikro ~0,5 mA).
- **Gemeinsame GND-Fläche** (wie in `power_design.md`).
- **ADC-Pin: GPIO7 (ADC1_CH6)** — frei (GPIO4/5 = Servos, GPIO6 = Akku-Teiler); bewusst **ADC1**, weil **ADC2** bei aktivem Wi-Fi/BLE blockiert ist; GPIO7 ist kein Strapping-Pin.
- Referenz: `power_design.md` (Versorgung, Masse, DevKit-Sockel).

## 8. LCSC-Suchbegriffe

| Bauteil | Suchbegriff | Filter |
|---|---|---|
| OpAmp | `MCP602-I/SN` (LCSC C29605) / `MCP6002` | SOIC-8, Rail-to-Rail I/O |
| Schottky | `BAT54` (SOD-523F, LCSC C283259) / `BAT43` / `BAT43W` | Kleinsignal, VF ≈ 0,3 V |
| Mikrofon-Stecker | `XH 2P` | JST-XH 2-pol |
| Elektret-Mikrofon | `GMI9745P-2C44DB` (INGHAi, LCSC C530035) | 9,7 mm, omnidirektional, −44 dB, 2-pol, **THT/Plug-in, 2,54 mm** |
| R 2,2k / 10k / 1,5k / 150k / 100k | `2.2k 0805 1%` … | 0805, 1 % |
| C 10nF / 100nF | `10nF 0805 50V` / `100nF 0805 50V` | X7R (C7 ideal C0G) |

## 9. Hinweise / Design-Entscheidungen

- **Offset × Verstärkung (gelöst):** Ohne Gegenmaßnahme wäre der Ruhepegel = Vos × 101.
  MCP6002: Vos typ ~1 mV, max ±4,5 mV → bis ~450 mV. MCP602: Vos typ ±0,7 mV, max ±2 mV → bis ~200 mV,
  dafür 2,8 MHz GBW (vs. 1 MHz beim MCP6002). **Fix: C8 (100 nF) in Reihe zu R6 → DC-Gain = 1** →
  der Offset-Anteil am Ausgang liegt dann nur noch bei ~Vos (mV-Bereich) statt ×101.
- **BAT43/BAT54 statt 1N5819:** Die 1N5819 (1-A-Netzteil-Diode) hat ~110 pF Sperrschichtkapazität und
  hohen Rückwärts-Leckstrom — die Ladung in C9 würde rückwärts durch die „sperrende" Diode abfließen.
  BAT43 (DO-35) / BAT43W (SOD-123) / BAT54 (SOT-23/SOD-523F) sind Kleinsignal-Schottkys (~7–10 pF, minimaler Leckstrom).
- **Sättigung ist ok:** Bei einem lauten Knall klippt der Verstärker an der Rail; für eine reine
  Schwellen-Erkennung unkritisch.
- **Mikrofon-Varianten:** entweder MIC1 (Kapsel auf der Platine) **oder** CN-Elektret (externe Kapsel)
  bestücken — nicht beide gleichzeitig (teilen sich sonst den Bias-Strom über R4).
- Optional: Ferrit/RC vor +3V3 der Analogsektion für saubereres AVCC (bei Schwellen-Trigger meist nicht nötig).

## 10. Schaltplan & Verdrahtung (EasyEDA-Umsetzung)

**Signalfluss:**

```
MIC1 / CN-Elektret ──► [C7/R5 Hochpass] ──► [U3A Verstärker] ──► [D1/C9/R1 Peak] ──► ADC (GPIO7)
Elektret              fc ≈ 1,6 kHz            AC ×101 / DC ×1        τ ≈ 1 ms           0–3,1 V
```

**Komponenten-Verdrahtung (Pin → Netz):**

| Bauteil | Pin | Ziel-Netz |
|---|---|---|
| MIC1 | + | `MIC_RAW` |
| MIC1 | − | `GND` |
| CN-Elektret | 1 | `MIC_RAW` |
| CN-Elektret | 2 | `GND` |
| CN-Jumper | 1 | `+3V3` |
| CN-Jumper | 2 | `R4.1` |
| R4 2,2k | 1 | `CN-Jumper.2` |
| R4 2,2k | 2 | `MIC_RAW` |
| C7 10n | 1 | `MIC_RAW` |
| C7 10n | 2 | `AMP_IN_P` |
| R5 10k | 1 | `GND` |
| R5 10k | 2 | `AMP_IN_P` |
| U3 MCP602 | 1 OUTA | `AMP_OUT` |
| U3 | 2 INA− | `AMP_IN_N` |
| U3 | 3 INA+ | `AMP_IN_P` |
| U3 | 4 VSS | `GND` |
| U3 | 5 INB+ | `GND` |
| U3 | 6 INB− | `OPAMP_B_PARK` (→ Pin 7) |
| U3 | 7 OUTB | `OPAMP_B_PARK` (→ Pin 6) |
| U3 | 8 VDD | `+3V3` |
| R6 1,5k | 1 | `GAIN_RC` |
| R6 1,5k | 2 | `AMP_IN_N` |
| C8 100n | 1 | `GND` |
| C8 100n | 2 | `GAIN_RC` |
| R7 150k | 1 | `AMP_OUT` |
| R7 150k | 2 | `AMP_IN_N` |
| D1 BAT54 | Anode (D1.2) | `AMP_OUT` |
| D1 BAT54 | Kathode (D1.1) | `ADC_TRIG_OUT` |
| C9 10n | 1 | `ADC_TRIG_OUT` |
| C9 10n | 2 | `GND` |
| R1 100k | 1 | `ADC_TRIG_OUT` |
| R1 100k | 2 | `GND` |
| C6 100n | 1 | `+3V3` |
| C6 100n | 2 | `GND` |

> **EasyEDA-Tipp:** Netze als **benannte Labels** anlegen (`+3V3`, `GND`, `MIC_RAW`, `AMP_IN_P`,
> `AMP_IN_N`, `GAIN_RC`, `AMP_OUT`, `ADC_TRIG_OUT`) — gleiche Labelnamen sind automatisch verbunden,
> du musst keine langen Leitungen ziehen. Netznamen entsprechen §4.

