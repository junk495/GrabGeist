# AGENTS.md — Projektleitfaden für KI-Agenten

## Projekt
**HostageTarget**: dynamisches Übungsziel, das hinter einem Pfosten hervor „peekt".
Mechanik: symmetrisches, ebenes 2-DOF-5-Gelenk-Parallelgetriebe (Fünfgelenkgetriebe), angetrieben von 2 Servos.

> **Autoritative Spezifikation:** `SystemSpecification.md`
> Bei Widersprüchen gilt die Spec; bei Unklarheit nachfragen statt raten.

## Tech-Stack
- **MCU:** ESP32-S3 (`esp32-s3-devkitc1-n16r8`), Arduino-Framework
- **Build:** PlatformIO (pioarduino-Fork — die offizielle Plattform unterstützt dieses Board nicht)
- **C++:** Arduino Core 3.x → gnu++17
- **Unit-Tests:** Unity, nativ (`platform = native`)

## Zentrale Maße (als leicht anpassbare Konstanten, Einheit mm)
| Konstante | Wert | Bedeutung |
|-----------|------|-----------|
| `D`  | 65  | Abstand der beiden Servoachsen |
| `L1` | 90  | Kurbellänge (Kurbelarm) |
| `L2` | 120 | Schubstangenlänge |

Alle Maße werden als **eine zentrale, leicht anpassbare Konstante** (`constexpr float`) geführt — nicht verstreut hartkodieren.

## Verzeichnisstruktur
- `src/main.cpp` — Arduino-Sketch (Einstieg, `setup()`/`loop()`, 50-Hz-Loop)
- `src/BleUart.{h,cpp}` — BLE-UART (Nordic-UART-Service) für die Handy-Teststeuerung (ESP32-spezifisch)
- `lib/Kinematics/` — inverse Kinematik (pur, hardwarefrei)
- `lib/ServoController/` — Kurbelwinkel → PWM-Pulsbreite
- `lib/MotionPlanner/` — nicht-blockierende Bahnplanung (Sprung/Sweep)
- `lib/CommandParser/` — Textbefehle → strukturierter `Command`
- `test/` — native Unit-Tests (Unity)
- `tools/test_gui.html` — Handy-Testoberfläche (Web Bluetooth)
- `SystemSpecification.md` — Spezifikation (autoritativ)
- `kinematik_konzept.md` — IK-Briefing (Maße, Formeln, Beispiel)
- `steuerung_konzept.md` — Steuerungskonzept (Tasten/BLE/Joystick/Web)
- `platformio.ini` — Build-/Test-Konfiguration

## Kommandos
- Build:  `pio run -e esp32s3`
- Upload: `pio run -e esp32s3 -t upload`
- Tests:  `pio test -e test`

## Konventionen
- Winkel intern in **Radiant**; Umrechnung auf Grad/PWM erst im `ServoController`.
- Datenfluss: `BleUart`/Eingabe → `CommandParser` → `MotionPlanner` → `Kinematics` → `ServoController` → LEDC.
- Wiederverwendbare/testbare Logik in `lib/`; ESP32-spezifischer Code (BLE, LEDC) in `src/`.
- IK-Solver `calculateIK(x, y, &thetaL, &thetaR)` muss ungültige Koordinaten mit `false` ablehnen (siehe Spec §4).
