# CAN-Bus-Schnittstelle — GrabGeist (PCB-Modul)

> Zweck: Anbindung des GrabGeist an einen **CAN-Bus** (z. B. Steuerung/Auslösung durch andere
> Geräte). Physik-Layer über **SN65HVD230** (3,3-V-Transceiver) an den **TWAI**-Controller des
> ESP32-S3. Bus-Ausgang als **3-Pol-Stecker** (CAN-H / CAN-L / GND) mit **schaltbarer 120-Ω-
> Terminierung** und **TVS-/ESD-Schutz**.
>
> **Stand:** ins Gesamt-Schema integriert (EasyEDA). RefDes konsolidiert (siehe `power_design.md`).

## 1. Übersicht

```
ESP32-S3 (U1)          SN65HVD230 (U5)              Bus-Stecker (CN-CAN)
GPIO21 ──CAN-D──► U5.1 (D)           U5.7 (CANH) ──► CAN-H  (Pin 2)
GPIO47 ◄─CAN-R── U5.4 (R)           U5.6 (CANL) ──► CAN-L  (Pin 3)
                      U5.3 (VCC) ── +3V3             GND ───► GND   (Pin 1)
                      U5.2 (GND) ── GND
                      U5.8 (Rs)  ── GND
                      U5.5 (Vref) ── offen (NC)
```

## 2. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | LCSC | Funktion |
|---|---|---|---|---|
| U5 | **SN65HVD230DR** (3,3-V-CAN-Transceiver) | SOIC-8 | C12084 | CAN-PHY, high-speed bis 1 Mbit/s |
| C14 | 100 nF (X7R) | 0805 | — | Abblocken VCC — direkt an U5.3 |
| R8 | 120 Ω (1 %) | 0805 | — | Bus-Terminierung, in Serie mit Jumper |
| JP-CAN | 2-Pin-Stiftleiste (Jumper) | THT 2,54 mm | C42391660 | Terminierung ein-/ausschaltbar |
| CN-CAN | 3-Pol XH2.54 | THT 2,54 mm | C7429633 | GND / CAN-H / CAN-L (Potentialausgleich) |
| D2 | **PESD1CAN** (Nexperia) | SOT-23 | C15771 | TVS-Array gegen ESD/Überspannung |

## 3. SN65HVD230 — Pinbelegung (SOIC-8)

| Pin | Name | Funktion | Anschluss |
|---|---|---|---|
| 1 | D (TXD) | Treiber-Eingang | GPIO21 (`CAN-D`) |
| 2 | GND | Masse | GND |
| 3 | VCC | Versorgung 3,3 V | +3V3 (C13 → GND) |
| 4 | R (RXD) | Empfänger-Ausgang | GPIO47 (`CAN-R`) |
| 5 | Vref | Referenz-Ausgang (VCC/2) | **offen (NC)** |
| 6 | CANL | Bus Low | CAN-L |
| 7 | CANH | Bus High | CAN-H |
| 8 | Rs | Slope-/Modus-Eingang | GND (High-Speed-Modus) |

## 4. Netzliste / Verdrahtung

| Netz | Pins |
|---|---|
| +3V3 | U5.3 (VCC), C14.1 |
| GND | U5.2 (GND), U5.8 (Rs), C14.2, CN-CAN.1, D2.3 |
| CAN-D | U1.27 (GPIO21), U5.1 (D) |
| CAN-R | U1.28 (GPIO47), U5.4 (R) |
| CAN-H | U5.7 (CANH), CN-CAN.2, D2.1, R8.1 |
| CAN-L | U5.6 (CANL), CN-CAN.3, D2.2, JP-CAN.2 |
| (R8–JP-CAN) | R8.2, JP-CAN.1 — Reihenschaltung R8 + Jumper zwischen CAN-H/CAN-L |

## 5. Terminierung

- **R8 (120 Ω) in Reihe mit JP-CAN (2-Pin-Jumper)** zwischen CAN-H und CAN-L:
  - Jumper **gesteckt** → Bus terminiert (für **erstes/letztes** Gerät am Bus).
  - Jumper **offen** → nicht terminiert (für Geräte dazwischen).
- Bei nur zwei Geräten: beide Enden mit Jumper terminieren.

## 6. TVS-/ESD-Schutz

- **D2 (PESD1CAN, SOT-23)** direkt am Stecker CN-CAN:
  - Pin 1 → CAN-H
  - Pin 2 → CAN-L
  - Pin 3 (gemeinsame **Anode**/Mitte) → GND
- **Hinweis:** Pin 3 ist beim PESD1CAN die **gemeinsame Anode** (nicht Kathode) und gehört an GND.

## 7. Pinbelegung (Firmware)

| Funktion | ESP32-S3-Pin | Netz | Konstante (später) |
|---|---|---|---|
| CAN-D (TXD) | **GPIO21** | CAN-D | neu |
| CAN-R (RXD) | **GPIO47** | CAN-R | neu |

- ESP32-S3 stellt den **TWAI**-Controller („CAN") bereit; die Pins sind über die GPIO-Matrix frei wählbar (hier GPIO21/47).
- GPIO21/47 sind **keine Strapping-Pins**; GPIO47 ist `SPICLK_P` (differentieller Flash-Takt), auf dem WROOM-1-N16R8 aber frei (Single-Ended-Takt).
- Achtung: Die Footprint-Pin-Nummern des DevKit-Symbols entsprechen **nicht** den GPIO-Nummern (z. B. GPIO1 = Pin 41) — im Symbol prüfen.

## 8. LCSC-Suchbegriffe

| Bauteil | Suchbegriff | Filter |
|---|---|---|
| CAN-Transceiver | `SN65HVD230` | SOIC-8, 3,3 V |
| TVS-Array | `PESD1CAN` / `NUP2105L` | SOT-23 |
| R 120 Ω | `120R 0805 1%` | 0805, 1 % |
| C 100 nF | `100nF 0805 50V` | X7R |
| Stecker 3-pol | `screw terminal 3P 3.5mm` oder `XH 3P` / `header 3P 2.54mm` | |
| Jumper 2-pol | `header 2P 2.54mm` | |

## 9. Hinweise / Design-Entscheidungen

- **Rs (U5.8) an GND** = High-Speed-Modus (max. Flankensteilheit, bis 1 Mbit/s). Bei EMI-Problemen
  an langen Leitungen optional einen Slope-Widerstand (z. B. 10–47 kΩ) von Rs nach GND einsetzen.
- **Vref (U5.5) offen lassen** — nur nötig, wenn ein externer Common-Mode-Bezug aufgebaut wird.
- **GND am Stecker ist zwingend** (Potentialausgleich); CAN braucht eine gemeinsame Masse aller Knoten.
- **Nur die beiden Bus-Enden terminieren** (Jumper), alle Zwischenknoten ohne 120 Ω.
- Optional für raue Umgebung: zusätzlich einen **Common-Mode-Choke** in Serie zu CAN-H/CAN-L
  (z. B. ACT45B) — für den Normalfall nicht nötig.
- Zusätzlich zum 100 nF optional ein 10 µF-Bulk auf +3V3 in der Nähe von U5 — bei sauberem 3V3 nicht nötig.
