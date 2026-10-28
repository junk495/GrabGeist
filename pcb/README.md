# PCB — GrabGeist

> Ziel: Der GrabGeist als **eigene Platine** — die saubere Umsetzung dessen, was der
> Probeaufbau (siehe `../verkabelung.md`, `../kinematik_konzept.md`, `../steuerung_konzept.md`)
> als lose Verdrahtung gezeigt hat. Dieses Verzeichnis wächst mit jedem neuen Modul.

## Status

**Schema steht (EasyEDA).** Stromversorgung und Akustik-Trigger sind zu einer **konsolidierten
Netzliste** zusammengeführt (RefDes übergreifend). Offen: Layout/Routing sowie die offenen Punkte
aus `power_design.md` §10.

## Module

| Modul | Datei | Status |
|---|---|---|
| ESP32-S3-DevKitC-1 (Controller) | [`esp32_devkit.md`](esp32_devkit.md) | Schema |
| Stromversorgung (2S2P-LiPo, SMD-Buck) | [`power_design.md`](power_design.md) | Schema |
| Akustischer Schussknall-Trigger | [`akustik_trigger.md`](akustik_trigger.md) | Schema |
| CAN-Bus-Schnittstelle (SN65HVD230) | [`can_bus.md`](can_bus.md) | Schema |
| Zusatz-Servo SG90 (5-V-Mikroservo) | [`sg90_servo.md`](sg90_servo.md) | Schema |
| Buzzer (Melodie, GPIO38) | [`buzzer.md`](buzzer.md) | Schema |
| Weitere Funktionen (z. B. PIR, Tasten) | — | geplant (Kandidaten aus der README-Roadmap) |

## Grundsätze der Platine

- **Alles SMD**; nur das **ESP32-S3 DevKitC-1 (N16R8)** wird gesteckt.
- Servos direkt am LiPo, Logik über einen diskreten 5V-Buck (siehe `power_design.md`).
- Jede neue Funktion erhält ein **eigenes Modul-Dokument** in diesem Verzeichnis.

## Abgrenzung zu den übrigen Doku-Dateien

- `verkabelung.md`, `kinematik_konzept.md`, `steuerung_konzept.md` beschreiben den
  **Ist-/Probeaufbau** und die Konzepte — bleiben als Funktions-Referenz bestehen.
- `SystemSpecification.md` bleibt autoritativ für Kinematik/Steuerung.
- `pcb/` beschreibt die **Ziel-Hardware** (Platine).

## Offene Punkte

- Modul-Liste ergänzen, sobald die weiteren Funktionen entschieden sind.
- Offene Punkte der einzelnen Module: siehe je Modul (z. B. `power_design.md` §10).
