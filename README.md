# HostageTarget

Dynamisches Übungsziel, das hinter einem Pfosten hervor „peekt" – angetrieben von einem ebenen
5-Gelenk-Parallelgetriebe (Fünfgelenkgetriebe) mit **2 Servos**.

## Kernidee

Der Nutzer steuert **nicht** die beiden Servos, sondern die **Zielposition** des zentralen
Lagerpunkts. Eine Handy-Oberfläche zeigt den tatsächlich erreichbaren – **nierenförmigen** –
Arbeitsraum; ein Tipp auf die Karte lässt das Ziel dorthin fahren. Die Firmware berechnet die
Inverse Kinematik in Echtzeit und weicht Hindernissen (den Servogehäusen) automatisch aus.

## Mechanik

| Größe | Wert |
|---|---|
| Servoabstand `D` | 65 mm |
| Kurbellänge `L1` | 90 mm |
| Schubstangenlänge `L2` | 125 mm |

- 2× 25kg Digital-RC-Servos (270° Stellbereich)
- Mechanik konstruiert in Fusion 360 – Servo-Grenzen & Kollisionsparameter sind bis zur
  finalen Vermessung **Platzhalter** (zentral als anpassbare Konstanten geführt).

## Features

- **Bimodale Inverse Kinematik** – „Knie nach außen" + „Over-Center" (>180°), 4 Kandidaten-Paare,
  7 Kollisionsprüfungen
- **Nierenförmiger Arbeitsraum** – obere Kalotte + untere Seitentaschen, Mitte unten gesperrt
  (Servogehäuse)
- **Workspace** – Raster-Karte, Snapping auf den nächsten gültigen Punkt, A*-Pfadplanung
- **Tap-to-Move** – Web-Bluetooth-Oberfläche rendert die Karte; Antippen = Zielposition
- **Hysterese** – verhindert Konfigurationssprünge der Servos während der Bewegung
- **BLE-Steuerung** – Textbefehle (`MOVE`, `PEEK`, `STOP`, `RAW`, `AUTO`, `MAP`, …)

## Tech-Stack

- ESP32-S3 (`esp32-s3-devkitc1-n16r8`), Arduino-Framework
- PlatformIO (pioarduino-Fork), C++17
- Unit-Tests mit Unity (nativ, `pio test -e test`)

## Build & Tests

```bash
pio run -e esp32s3            # Build
pio run -e esp32s3 -t upload  # Flashen
pio test -e test              # Unit-Tests (nativ)
```

## Benutzung

1. Firmware auf den ESP32-S3 flashen.
2. `tools/test_gui.html` im Browser öffnen (Android: Chrome, Web Bluetooth).
3. „Verbinden" → Gerät **„HostageTarget"** wählen.
4. Auf die grüne Karte tippen → das Ziel fährt dorthin (der ESP32 sucht den Weg um die Sperrzone).

## Dokumentation

| Datei | Inhalt |
|---|---|
| `SystemSpecification.md` | autoritative Spezifikation |
| `kinematik_konzept.md` | IK-Briefing (Formeln, Beispiele) |
| `steuerung_konzept.md` | Steuerungskonzept (Tasten/BLE/Joystick) |

## Lizenz

[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) – Namensnennung, nicht-kommerziell, Weitergabe unter gleichen Bedingungen. Details siehe `LICENSE`.
