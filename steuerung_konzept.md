# Steuerungskonzept — GrabGeist

> Ergänzt `SystemSpecification.md` (§5.4 Betriebsmodi) um die Frage:
> **Wie bedient der Anwender den Geist?** (Tasten, Joystick, Handy, Web-UI)

## 1. Ziel & Ausgangslage

Der GrabGeist läuft meist **autonom** (`AUTO`/`RANDOM`) und schaut in unregelmäßigen
Abständen hinter dem Grabstein hervor. Wer ihn **live** steuern will (Handy oder Taster),
tut das meist „blind" – während man auf die Reaktion der Besucher achtet.

Daraus folgt die wichtigste Regel:

> **Live-Steuerung muss „blind" (taktil) bedienbar sein. Ein Touchscreen ist dafür
> ungeeignet, weil man hinschauen muss. Touchscreen/App eignet sich für Setup & Parameter.**

## 2. Bewertung der Eingabemöglichkeiten

| Option | Live-Steuerung | Stärke | Schwäche |
|---|---|---|---|
| **Tasten** | ✅ sehr gut | blind bedienbar, instant, zuverlässig, billig | nur diskrete Positionen |
| **Joystick (analog)** | ✅ gut | stufenlos, direkte X/Y-Mapping, intuitiv | 2× ADC + Deadzone, meist Kabel |
| **Handy (BLE)** | ⚠️ mittel | drahtlos, schöne UI, kein Zusatz-Hardware | Touchscreen = hinschauen, Latenz, Pairing |
| **Web-UI (WiFi)** | ⚠️ mittel | jeder Browser, ohne App | mehr Code, Netz-Setup, Latenz |

## 3. Empfehlung: Schichtenmodell

Die Optionen ergänzen sich – **nicht** entweder/oder. Alle Eingaben laufen am Ende
über **eine gemeinsame Befehlsschnittstelle** (siehe §4).

| Schicht | Bestandteil | Priorität |
|---|---|---|
| **1. Basis** | 4–5 **Tasten** (Links/Rechts/Oben/Versteckt + Modus) | Pflicht |
| **2. Komfort** | **Handy via BLE** (Moduswahl, Parameter, Live-X/Y) | empfohlen |
| **3. Optional** | **Joystick** (analog, stufenlos + RECORD-Aufnahme) | später |
| **4. Optional** | **Web-UI** (WiFi) für Konfiguration | nur bei Bedarf |

**Warum Tasten als Basis?**
- Sofortige Reaktion, zuverlässig, blind bedienbar.
- Praktisch: eine **physische „Verstecken/Stopp"-Taste**, um den Geist sofort wieder zu
  verstecken.

**Warum BLE statt WiFi fürs Handy?**
- Direkt Gerät↔Handy, keine Netz-Konfiguration, niedrige Latenz.
- ESP32-S3 hat BLE an Bord (kostet nichts extra). Als **BLE-UART-Service** umsetzbar,
  dann genügt eine generische App (z.B. „Serial Bluetooth Terminal") oder später eine Mini-App.

## 4. Einheitliche Befehlsschnittstelle

Ein Befehl pro Zeile (ASCII, Zeilenumbruch-terminiert). Wird **identisch** über
Serial (Debug) und BLE-UART verarbeitet. Tasten/Joystick erzeugen intern dieselben Befehle.

```
MOVE <x> <y> [snap]        # Zielposition in mm (Default: smooth + A*-Pfad)
PEEK <left|right|top|hide> # Preset-Kurzformen
RAW <l> <r>                # direkte Servo-Pulsbreiten [us] (500..2500)
STOP                       # sofort verstecken / anhalten
STATUS                     # aktueller Zustand ausgeben

AUTO                       # fester Ablauf (links -> oben -> rechts -> versteckt)
RANDOM                     # autonomer Zufallsmodus (toggle)
REC                        # Aufnahme starten/stoppen (20 Hz)
PLAY                       # Aufnahme als Schleife abspielen (toggle)

RAMP [<us/s>]              # Servo-Rampe setzen/abfragen
SPEED [<%>]                # Geschwindigkeit setzen/abfragen (skaliert unter der Rampe)
RESETLOG                   # Grund des letzten Resets ausgeben
MAP                        # Arbeitsraum-Karte anfordern
WIFI <on|off|auto>         # WiFi-SoftAP (Web-UI) steuern

# Abläufe (Sequenzen) werden über die JSON-API der Web-UI editiert:
#   GET/POST /api/sequence  (JSON-Ablauf laden/speichern, persistent in /sequence.json)
#   POST      /api/run      (Ablauf abspielen)
#   GET       /api/map, /api/status, /api/presets, /api/config
```

**Konsequenz für die Firmware:**
- Neues Modul `ModeManager` (bzw. `Command`-Parser) sitzt **über** den bestehenden Klassen
  (`Kinematics`, `ServoController`, `ServoSpeed`).
- Jede Eingabe (Taste, BLE, Joystick, Web) liefert nur Befehle → Eingaben sind
  austauschbar, ohne die Kernlogik anzufassen.

## 5. Tastenbelegung (Vorschlag)

GPIOs ohne Konflikt mit Boot-Strapping (0/3/45/46), UART0 (43/44), USB (19/20),
Flash/PSRAM (26–37). Die Servo-Pins 4/5 bleiben frei.

| Taste | GPIO | Befehl |
|---|---|---|
| Links-Peek | 6 | `PEEK left` |
| Rechts-Peek | 7 | `PEEK right` |
| Oben-Peek | 8 | `PEEK top` |
| Verstecken | 9 | `PEEK hide` (sollte **snap** sein → sofort weg) |
| Modus | 10 | zyklisch: RANDOM → MANUAL → PRESET → RANDOM → … |

Hinweise:
- Tasten an GPIO mit internem **Pull-up**, aktive Flanke auf LOW (gegen GND), **Entprellung**
  (z.B. 20–50 ms) erforderlich.
- Optionaler **analoger Joystick**: 2× **ADC1** (GPIO1–10, z.B. GPIO1/GPIO2), da ADC1 –
  anders als ADC2 – auch bei aktiver Funk-Schnittstelle zuverlässig bleibt.

## 6. Handy-Anbindung (BLE)

1. ESP32-S3 als **BLE-Peripheral** mit **Nordic-UART-Service** (TX/RX-Charakteristik).
2. Handy sendet Zeilen-Befehle (§4) und empfängt `STATUS`/Log-Ausgaben.
3. Stufenplan: erst generische BLE-Terminal-App, später (optional) kleine eigene App.

## 7. Umsetzungsreihenfolge

1. **Phase 1 (Basis):** `ModeManager`/`Command`-Parser + **Tasten** + Presets.
2. **Phase 2:** **BLE-UART** fürs Handy (Moduswahl + Parameter).
3. **Phase 3 (optional):** analoger **Joystick** (MANUAL/Sweep, RECORD) und/oder Web-UI.

**Aktueller Stand:** `CommandParser`, **BLE-UART** (`src/BleUart`), **WiFi-SoftAP + JSON-API** (`src/ApiServer`, Web-UI `data/index.html`), `Kinematics` (bimodal + Kollision + Hysterese), `Workspace` (Karte/Snap/A* + Sperrzonen-Abstand), **Tap-to-Move**, `ServoController`, `ServoSpeed` (Servo-Rampe/Strombegrenzung), **Reset-Log (NVS)**, **Sequence** (JSON-Ablauf, persistent in `/sequence.json`) sowie die Modi `AUTO`, `RANDOM`, `REC`/`PLAY` sind umgesetzt. Noch offen: `ModeManager`/**Tasten** (Phase 1).

## 8. Offene Punkte (hängen von der Mechanik ab)

- Exakte **Preset-Koordinaten** für `PEEK left/right/top/hide` → aus Fusion/CAD ableiten.
- **„Versteckt"-Position**: wo verschwindet der Geist sicher hinter dem Grabstein?
- Eigene App: **ja/nein** (erst mit generischer BLE-Terminal-App starten).
- Parameter-Set für `PARAM` (Dauern, Geschwindigkeiten, RANDOM-Bereiche) finalisieren.
