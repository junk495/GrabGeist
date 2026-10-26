# Akustischer Schussknall-Trigger — GrabGeist (PCB-Modul)

> Zweck: analoge Aufbereitung eines akustischen Impulses (Schussknall / Kugelschlag in
> 10–25 m Distanz) zur Auslösung eines Ziels über den ADC des ESP32-S3.
> Versorgung **+3,3 V** (vom DevKit-LDO), Ruhepegel am Ausgang **~0 V**
> (nur die positiven Halbwellen werden verstärkt = Single-Supply-Gleichrichtung).

## 1. Signalweg

1. Elektret-Mikrofon mit DC-Bias (R1 nach +3V3).
2. RC-Hochpass 1. Ordnung (C1/R2, fc ≈ 1,59 kHz) — filtert Wind, Trittschall, Motorenbrumm.
3. Nicht-invertierender Verstärker (U1A): **AC-Gain G = 1 + R4/R3 = 101 ≈ 40 dB, DC-Gain = 1**
   (durch C4 in Reihe zu R3). R3 + C4 bilden einen **2. Hochpass** (fc ≈ 1,06 kHz).
4. Spitzenwert-/Hüllkurvendetektor (D1, C2, R5, τ = R5·C2 = 1 ms) — dehnt µs-Impulse auf 1–3 ms,
   damit einfaches ADC-Polling (2–4 kHz) ohne Hardware-Interrupt genügt.

## 2. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | Funktion |
|---|---|---|---|
| U1 | **MCP602-I/SN** (LCSC C29605; alternativ MCP6002) | SOIC-8 | Dual Rail-to-Rail-I/O-OpAmp, 2,8 MHz GBW, 3,3-V-tauglich; **kein** LM358 |
| MIC1 | Elektret-Kapsel (extern) / 2-Pin-Header | JST-XH 2-pol | Mikrofon, mechanisch vom Gehäuse entkoppelt |
| JP1 | Lötbrücke / Jumper (optional) | SMD | in Serie zu R1 (trennt Bias, falls Piezo statt Elektret) |
| D1 | **BAT54** (SOD-523F, LCSC C283259) — alternativ BAT43/BAT43W | SOD-523F / DO-35 / SOD-123 | Kleinsignal-Schottky (40 V, 300 mA), Spitzenwertgleichrichter |
| R1 | 2,2 kΩ | 0805 | Mikrofon-Bias nach +3V3 |
| R2 | 10 kΩ | 0805 | Hochpass + DC-Arbeitspunkt (0 V) für INA+ |
| R3 | 1,5 kΩ | 0805 | Gain-Fußpunkt (AC) — in Serie mit C4 nach GND |
| R4 | 150 kΩ | 0805 | Gegenkopplung (G = 1 + R4/R3 = 101) |
| R5 | 100 kΩ | 0805 | Entladewiderstand der Hüllkurve (τ = 1 ms) |
| C1 | 10 nF (X7R/C0G) | 0805 | DC-Entkopplung + Hochpass (fc = 1,59 kHz) |
| C2 | 10 nF (X7R) | 0805 | Haltekondensator (τ = R5·C2 = 1 ms) |
| C3 | 100 nF (X7R) | 0805 | Decoupling für U1 (nah an Pin 8) |
| C4 | 100 nF (X7R) | 0805 | in Reihe zu R3 → DC-Gain = 1 + 2. Hochpass (fc ≈ 1,06 kHz) |

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
| +3V3 | U1.8 (VDD), C3.1, R1.1 (optional über JP1) |
| GND | U1.4 (VSS), U1.5 (INB+), C3.2, C4.2, MIC1.2, R2.2, C2.2, R5.2 |
| MIC_RAW | MIC1.1, R1.2, C1.1 |
| AMP_IN_P | C1.2, R2.1, U1.3 (INA+) |
| AMP_IN_N | U1.2 (INA−), R3.1, R4.1 |
| GAIN_RC | R3.2, C4.1 — R3 nicht direkt an GND, sondern über C4 |
| AMP_OUT | U1.1 (OUTA), R4.2, D1.Anode |
| ADC_TRIG_OUT | D1.Kathode, C2.1, R5.1, MCU-ADC (GPIO7) |
| OPAMP_B_PARK | U1.6 (INB−), U1.7 (OUTB) — ungenutzten Kanal als Follower terminieren |

## 5. Layout-Regeln

- C3 so nah wie möglich zwischen U1 Pin 8 (VDD) und GND.
- AMP_IN_N / AMP_IN_P (U1 Pin 2/3) extrem kurz führen; **nicht** parallel zu PWM-/Servo-/Buck-Leitungen.
- Analoge Sektion über geschlossener GND-Fläche, abseits der Servo-Rückströme.
- MIC1 als 2-poligen Steckverbinder (JST-XH) ausführen; Kapsel über kurzes verdrilltes Kabel in
  Schaumstoff lagern (Körper-/Windentkopplung). Die GMI9745P-2C44DB hat **THT-Pins (2,54 mm)** —
  ideal: Kabel vom JST-XH-Stecker mit einer 2-poligen Buchse (oder Lötverbindung) an die Pins führen.

## 6. Firmware-Schnittstelle (später)

- Signal: 0 … 3,1 V an ADC_TRIG_OUT.
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
| Elektret-Mikrofon (extern) | `GMI9745P-2C44DB` (INGHAi, LCSC C530035) | 9,7 mm, omnidirektional, −44 dB, 2-pol, **THT/Plug-in, 2,54 mm** |
| R 2,2k / 10k / 1,5k / 150k / 100k | `2.2k 0805 1%` … | 0805, 1 % |
| C 10nF / 100nF | `10nF 0805 50V` / `100nF 0805 50V` | X7R (C1 ideal C0G) |

## 9. Hinweise / Design-Entscheidungen

- **Offset × Verstärkung (gelöst):** Ohne Gegenmaßnahme wäre der Ruhepegel = Vos × 101.
  MCP6002: Vos typ ~1 mV, max ±4,5 mV → bis ~450 mV. MCP602: Vos typ ±0,7 mV, max ±2 mV → bis ~200 mV,
  dafür 2,8 MHz GBW (vs. 1 MHz beim MCP6002). **Fix: C4 (100 nF) in Reihe zu R3 → DC-Gain = 1** →
  der Offset-Anteil am Ausgang liegt dann nur noch bei ~Vos (mV-Bereich) statt ×101.
- **BAT43/BAT54 statt 1N5819:** Die 1N5819 (1-A-Netzteil-Diode) hat ~110 pF Sperrschichtkapazität und
  hohen Rückwärts-Leckstrom — die Ladung in C2 würde rückwärts durch die „sperrende" Diode abfließen.
  BAT43 (DO-35) / BAT43W (SOD-123) / BAT54 (SOT-23/SOD-523F) sind Kleinsignal-Schottkys (~7–10 pF, minimaler Leckstrom).
- **Sättigung ist ok:** Bei einem lauten Knall klippt der Verstärker an der Rail; für eine reine
  Schwellen-Erkennung unkritisch.
- Optional: Ferrit/RC vor +3V3 der Analogsektion für saubereres AVCC (bei Schwellen-Trigger meist nicht nötig).

## 10. Schaltplan & Verdrahtung (EasyEDA-Umsetzung)

**Signalfluss:**

```
MIC1 ──► [C1/R2 Hochpass] ──► [U1A Verstärker] ──► [D1/C2/R5 Peak] ──► ADC (GPIO7)
Elektret   fc ≈ 1,6 kHz         AC ×101 / DC ×1        τ ≈ 1 ms          0–3,1 V
```

**Komponenten-Verdrahtung (Pin → Netz):**

| Bauteil | Pin | Ziel-Netz |
|---|---|---|
| MIC1 | + | `MIC_RAW` |
| MIC1 | − | `GND` |
| R1 2,2k | 1 | `+3V3` (über JP1) |
| R1 2,2k | 2 | `MIC_RAW` |
| C1 10n | 1 | `MIC_RAW` |
| C1 10n | 2 | `AMP_IN_P` |
| R2 10k | 1 | `AMP_IN_P` |
| R2 10k | 2 | `GND` |
| U1 MCP602 | 1 OUTA | `AMP_OUT` |
| U1 | 2 INA− | `AMP_IN_N` |
| U1 | 3 INA+ | `AMP_IN_P` |
| U1 | 4 VSS | `GND` |
| U1 | 5 INB+ | `GND` |
| U1 | 6 INB− | `OPAMP_B_PARK` (→ Pin 7) |
| U1 | 7 OUTB | `OPAMP_B_PARK` (→ Pin 6) |
| U1 | 8 VDD | `+3V3` |
| R3 1,5k | 1 | `AMP_IN_N` |
| R3 1,5k | 2 | `GAIN_RC` |
| C4 100n | 1 | `GAIN_RC` |
| C4 100n | 2 | `GND` |
| R4 150k | 1 | `AMP_IN_N` |
| R4 150k | 2 | `AMP_OUT` |
| D1 BAT54 | Anode | `AMP_OUT` |
| D1 BAT54 | Kathode | `ADC_TRIG_OUT` |
| C2 10n | 1 | `ADC_TRIG_OUT` |
| C2 10n | 2 | `GND` |
| R5 100k | 1 | `ADC_TRIG_OUT` |
| R5 100k | 2 | `GND` |
| C3 100n | 1 | `+3V3` |
| C3 100n | 2 | `GND` |

> **EasyEDA-Tipp:** Netze als **benannte Labels** anlegen (`+3V3`, `GND`, `MIC_RAW`, `AMP_IN_P`,
> `AMP_IN_N`, `GAIN_RC`, `AMP_OUT`, `ADC_TRIG_OUT`) — gleiche Labelnamen sind automatisch verbunden,
> du musst keine langen Leitungen ziehen. Netznamen entsprechen §4.

