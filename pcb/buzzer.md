# Buzzer (Melodie) — GrabGeist (PCB-Modul)

> Zweck: passiver elektromagnetischer Buzzer für Töne/Melodien (z. B. „Akku leer"-Jingle,
> Trigger-Feedback). Angesteuert über **LEDC-PWM** an **GPIO38**. Da der Buzzer 100 mA zieht und
> induktiv ist, wird er über eine **NPN-Transistor-Stufe + Freilaufdiode** getrieben.

## 1. Bauteilliste (BOM)

| Ref | Bauteil / Wert | Gehäuse | LCSC | Funktion |
|---|---|---|---|---|
| BUZZER1 | MLT-8540H (passiv, elektromagnetisch, 2,7 kHz) | SMD 8,5×8,5 mm | C95298 | Buzzer |
| Q2 | SS8050 (NPN, 25 V / 1,5 A) | SOT-23 | C7420370 | Treiber |
| D3 | 1N4148W (100 V / 150 mA, trr 4 ns) | SOD-123 | C7420318 | Freilaufdiode |
| R9 | 1 kΩ | 0805 | C95781 | Basis-Widerstand |

## 2. Schaltung / Netzliste

```
GPIO38 (U1.35) ── R9 (1k) ── Q2.Basis
+3V3 ── BUZZER1+ ── BUZZER1− ── Q2.Collector
                                Q2.Emitter ── GND
+3V3 ── D3.Kathode ── D3.Anode ── Q2.Collector   (D3 parallel zu BUZZER1)
```

| Netz | Pins |
|---|---|
| BUZ_CTRL | U1.35 (GPIO38), R9.2 |
| BUZ_BASE | R9.1, Q2.1 (Basis) |
| +3V3 | BUZZER1.1 (+), D3.1 (Kathode) |
| BUZ_SW | BUZZER1.2 (−), Q2.3 (Collector), D3.2 (Anode) |
| GND | Q2.2 (Emitter) |

> **SS8050 (SOT-23):** Pin 1 = Basis, Pin 2 = Emitter, Pin 3 = Collector — im EasyEDA-Footprint prüfen.
> **D3-Polarität:** Kathode (Ring) an +3V3, Anode an Collector.

## 3. Firmware (LEDC, Arduino Core 3.x)

- `ledcAttach(38, 1000, 8);` — einmalig.
- `ledcWriteTone(38, freq);` — Ton starten; `freq = 0` = aus.
- Resonanz **2,7 kHz** → Noten um **2–4 kHz** sind am lautesten.

## 4. Hinweise

- **Passiv** → Frequenz erzeugt die Firmware (beliebige Melodien).
- **D3 (1N4148W)** fängt die Induktionsspitze beim Abschalten (Schutz von Q2).
- Buzzer an **+3V3** (Nenn 3,6 V, Bereich 2,5–4,5 V).
- **BUZZER1 hat nur 2 funktionale Anschlüsse (Spule, 16 Ω):** `+` und `−`.
  Weitere Pads (im Footprint oft **Pin 3/4**) sind **NC/Montage-Pads** — nur löten (mechanisch),
  **nicht** elektrisch verdrahten. Datenblatt: „Terminal plate: 3 soldering pads".
