# GrabGeist

Ein Geist, der hinter einem Grabstein immer wieder hervorschaut – angetrieben von einem ebenen
5-Gelenk-Parallelgetriebe (Fünfgelenkgetriebe) mit **2 Servos**.

## Kernidee

Der Nutzer steuert **nicht** die beiden Servos, sondern die **Zielposition** des zentralen
Lagerpunkts. Eine Handy-Oberfläche zeigt den tatsächlich erreichbaren – **nierenförmigen** –
Arbeitsraum; ein Tipp auf die Karte lässt den Geist dorthin fahren. Die Firmware berechnet die
Inverse Kinematik in Echtzeit und weicht Hindernissen (den Servogehäusen) automatisch aus.

## Mechanik

| Größe | Wert |
|---|---|
| Servoabstand `D` | 65 mm |
| Kurbellänge `L1` | 90 mm |
| Schubstangenlänge `L2` | 125 mm |

- 2× 25kg Digital-RC-Servos (270° Stellbereich)
- Servo-Kalibrierung & Kollisionsparameter sind **am Modell verifiziert** (zentral als
  anpassbare Konstanten geführt).

## Features

- **Bimodale Inverse Kinematik** – „Knie nach außen" + „Over-Center" (>180°), 4 Kandidaten-Paare,
  7 Kollisionsprüfungen
- **Nierenförmiger Arbeitsraum** – obere Kalotte + untere Seitentaschen, Mitte unten gesperrt
  (Servogehäuse)
- **Workspace** – Raster-Karte, Snapping auf den nächsten gültigen Punkt, A*-Pfadplanung mit
  automatischem Sicherheitsabstand zur Sperrzone
- **Tap-to-Move** – Web-Bluetooth-Oberfläche rendert die Karte; Antippen = Zielposition
- **Servo-Anfahrrampe** – einstellbare Beschleunigungs-/Geschwindigkeitsbegrenzung
  (verhindert Stromspitzen/Brownouts)
- **Hysterese** – verhindert Konfigurationssprünge der Servos während der Bewegung
- **Betriebsmodi** – `AUTO` (fester Ablauf), `RANDOM` (zufälliges Hervorschauen),
  `REC`/`PLAY` (Aufnahme & Wiedergabe), Script-Abläufe als Datei
- **Reset-Diagnose** – Reset-Grund wird persistent in NVS gespeichert und per BLE abrufbar
- **BLE-Steuerung** – Textbefehle (`MOVE`, `PEEK`, `STOP`, `RAW`, `AUTO`, `RANDOM`, `RAMP`,
  `REC`, `PLAY`, `RUN`, …)

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
3. „Verbinden" → Gerät **„GrabGeist"** wählen.
4. Auf die grüne Karte tippen → der Geist fährt dorthin (der ESP32 sucht den Weg um die Sperrzone).
5. Weitere Bedienelemente:
   - **Peek-Buttons** (`Links`, `Rechts`, `Beide oben`, `Versteckt`) und **STOP**
   - **AUTO** (fester Ablauf) und **Zufall** (autonomes Hervorschauen)
   - **Servo-Tempo** (Anfahrrampe, live einstellbar)
   - **Aufnahme / Abspielen** (Bewegung aufnehmen und als Schleife abspielen)
   - **Script** (Ablauf als `.txt`-Datei hochladen und abspielen)
   - **Reset-Log** (Grund des letzten Resets anzeigen)

## Dokumentation

| Datei | Inhalt |
|---|---|
| `SystemSpecification.md` | autoritative Spezifikation |
| `kinematik_konzept.md` | IK-Briefing (Formeln, Beispiele) |
| `steuerung_konzept.md` | Steuerungskonzept (Tasten/BLE/Joystick) |
| `verkabelung.md` | Verkabelung & Stromversorgung |

## Roadmap

Ideen für die nächsten Schritte (noch nicht umgesetzt):

- **Persistenz + Auto-Start (NVS)** – Script, Parameter und aktiver Modus überleben einen
  Neustart; optional Autostart für den Standalone-Betrieb ohne Handy.
- **Bewegungsauslöser (PIR-Sensor)** – Geist reagiert auf Näherung.
- **Physische Taste(n)** – „Verstecken/Stopp" (blind bedienbar) und ggf. Peek-Tasten.
- **Parameter-Persistenz (NVS)** – Rampe/Zufalls-Dauern speichern.
- **JSON-Script-Format** – strukturierte Abläufe statt Zeilen-Befehlen.
- **GUI-Ausbau** – Modus-Anzeige, Live-Position, Rampe-Synchronisation.
- **Cleanup** – toten Code entfernen, optional `ModeManager` zur Strukturierung.

## Lizenz

[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) – Namensnennung, nicht-kommerziell, Weitergabe unter gleichen Bedingungen. Details siehe `LICENSE`.
