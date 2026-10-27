# CAN-Bus-Schnittstelle — GrabGeist (PCB-Modul)

> Zweck: Anbindung des GrabGeist an einen **CAN-Bus** (z. B. Steuerung/Auslösung durch andere
> Geräte). Physik-Layer über **SN65HVD230** (3,3-V-Transceiver) an den **TWAI**-Controller des
> ESP32-S3. Bus-Ausgang als **3-Pol-Stecker** (CAN_H / CAN_L / GND) mit **schaltbarer 120-Ω-
> Terminierung** und **TVS-/ESD-Schutz**.
>
> **Stand:** ins Gesamt-Schema integriert (EasyEDA). RefDes konsolidiert (siehe `power_design.md`).

## 1. Übersicht

```
ESP32-S3 (U1)          SN65HVD230 (U4)              Bus-Stecker (CN_CAN)
GPIO15 ──CAN_TX──► U4.1 (D)           U4.7 (CANH) ──► CAN_H  (Pin 1)
GPIO16 ◄─CAN_RX── U4.4 (R)           U4.6 (CANL) ──► CAN_L  (Pin 2)
                      U4.3 (VCC) ── +3V3             GND ───► GND   (Pin 3)
                      U4.2 (GND) ── GND
                      U4.8 (Rs)  ── GND
                      U4.5 (Vref) ── offen (NC)
```

## 2. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | Funktion |
|---|---|---|---|
| U4 | **SN65HVD230** (3,3-V-CAN-Transceiver) | SOIC-8 | CAN-PHY, high-speed bis 1 Mbit/s |
| C13 | 100 nF (X7R) | 0805 | Abblocken VCC — direkt an U4.3 |
| R8 | 120 Ω (1 %) | 0805 | Bus-Terminierung, in Serie mit Jumper |
| JP_CAN | 2-Pin-Stiftleiste (Jumper) | THT 2,54 mm | Terminierung ein-/ausschaltbar |
| CN_CAN | 3-Pol Schraubklemme (3,5 mm) oder Stiftleiste | THT | CAN_H / CAN_L / GND (Potentialausgleich) |
| D2 | **PESD1CAN** (Nexperia) oder **NUP2105L** (onsemi) | SOT-23 | TVS-Array gegen ESD/Überspannung |

## 3. SN65HVD230 — Pinbelegung (SOIC-8)

| Pin | Name | Funktion | Anschluss |
|---|---|---|---|
| 1 | D (TXD) | Treiber-Eingang | GPIO15 (`CAN_TX`) |
| 2 | GND | Masse | GND |
| 3 | VCC | Versorgung 3,3 V | +3V3 (C13 → GND) |
| 4 | R (RXD) | Empfänger-Ausgang | GPIO16 (`CAN_RX`) |
| 5 | Vref | Referenz-Ausgang (VCC/2) | **offen (NC)** |
| 6 | CANL | Bus Low | CAN_L |
| 7 | CANH | Bus High | CAN_H |
| 8 | Rs | Slope-/Modus-Eingang | GND (High-Speed-Modus) |

## 4. Netzliste / Verdrahtung

| Netz | Pins |
|---|---|
| +3V3 | U4.3 (VCC), C13.1 |
| GND | U4.2 (GND), U4.8 (Rs), C13.2, CN_CAN.3, D2.3 |
| CAN_TX | U1.15 (GPIO15), U4.1 (D) |
| CAN_RX | U1.16 (GPIO16), U4.4 (R) |
| CAN_H | U4.7 (CANH), CN_CAN.1, D2.1, R8.1 |
| CAN_L | U4.6 (CANL), CN_CAN.2, D2.2, JP_CAN.2 |
| CAN_TERM | R8.2, JP_CAN.1 — Reihenschaltung R8 + Jumper zwischen CAN_H/CAN_L |

## 5. Terminierung

- **R8 (120 Ω) in Reihe mit JP_CAN (2-Pin-Jumper)** zwischen CAN_H und CAN_L:
  - Jumper **gesteckt** → Bus terminiert (für **erstes/letztes** Gerät am Bus).
  - Jumper **offen** → nicht terminiert (für Geräte dazwischen).
- Bei nur zwei Geräten: beide Enden mit Jumper terminieren.

## 6. TVS-/ESD-Schutz

- **D2 (PESD1CAN / NUP2105L, SOT-23)** direkt am Stecker CN_CAN:
  - Pin 1 → CAN_H
  - Pin 2 → CAN_L
  - Pin 3 (gemeinsame **Anode**/Mitte) → GND
- **Hinweis:** Pin 3 ist beim PESD1CAN/NUP2105L die **gemeinsame Anode** (nicht Kathode) und gehört an GND.

## 7. Pinbelegung (Firmware)

| Funktion | ESP32-S3-Pin | Netz | Konstante (später) |
|---|---|---|---|
| CAN TX | **GPIO15** | CAN_TX | neu |
| CAN RX | **GPIO16** | CAN_RX | neu |

- ESP32-S3 stellt den **TWAI**-Controller („CAN") bereit; die Pins sind über die GPIO-Matrix frei wählbar (hier GPIO15/16).
- GPIO15/16 sind **keine Strapping-Pins** und aktuell frei (GPIO4/5 = Servos, GPIO6/7 = ADC).
- Pin-Nummern im Schema folgen der DevKit-Footprint-Konvention (GPIO-N = Pin-N, wie U1.4=GPIO4 …) — gegen das Footprint prüfen.

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

- **Rs (U4.8) an GND** = High-Speed-Modus (max. Flankensteilheit, bis 1 Mbit/s). Bei EMI-Problemen
  an langen Leitungen optional einen Slope-Widerstand (z. B. 10–47 kΩ) von Rs nach GND einsetzen.
- **Vref (U4.5) offen lassen** — nur nötig, wenn ein externer Common-Mode-Bezug aufgebaut wird.
- **GND am Stecker ist zwingend** (Potentialausgleich); CAN braucht eine gemeinsame Masse aller Knoten.
- **Nur die beiden Bus-Enden terminieren** (Jumper), alle Zwischenknoten ohne 120 Ω.
- Optional für raue Umgebung: zusätzlich einen **Common-Mode-Choke** in Serie zu CAN_H/CAN_L
  (z. B. ACT45B) — für den Normalfall nicht nötig.
- Zusätzlich zum 100 nF optional ein 10 µF-Bulk auf +3V3 in der Nähe von U4 — bei sauberem 3V3 nicht nötig.
