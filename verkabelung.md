# Verkabelung — GrabGeist (Probeaufbau)

> Gültig für den ersten Aufbau mit **ESP32-S3 DevKitC-1 (N16R8)** und
> **2× Miuzei MS62-2-D-B** (25kg-Digital-RC-Servo, 270°, 4,8–8,4 V, 500–2500 µs).
> Alle Bezeichnungen beziehen sich auf die Firmware in `src/main.cpp`.

## 1. Übersicht

Der ESP32 erzeugt nur die **Steuersignale** (PWM, 3,3 V-Logik). Die **Servo-Leistung**
(Spannung + Strom) kommt aus einer **separaten Stromversorgung** — niemals aus dem 3V3-Pin
des Entwicklungsboards.

```
  +-----------------------+          +---------------------+
  |  ESP32-S3 DevKitC-1   |          |   DPS150 @ 7,0 V, CC 5 A|
  |                       |          |                         |
  |   GPIO4  --------------- Signal --> linker  Servo (weiß)  |
  |   GPIO5  --------------- Signal --> rechter Servo (weiß)  |
  |                       |          |                         |
  |   GND   <============> GND (gemeinsame Masse, zwingend)   |
  |                       |          |     + ----+---- +      |
  |                       |          |      |         |       |
  |   (USB -> PC/Netz)    |          |     rot       rot      |
  |                       |          |      |         |       |
  +-----------------------+          |   Servo L   Servo R    |
                                     +---------------------+
```

## 2. Benötigte Materialien

| Pos | Bauteil | Hinweis |
|-----|---------|---------|
| 1 | ESP32-S3 DevKitC-1 (N16R8) | Logikpegel **3,3 V** |
| 2 | Miuzei MS62-2-D-B (25kg, 270°) | links + rechts, 4,8–8,4 V |
| 1 | Labornetzteil DPS150 | **separate** Spannungsquelle für die Servos (7,0 V, CC 5 A) |
| 1 | Bulk-/Stütz-Kondensator | 470–1000 µF, ≥ Spannung des Netzteils |
| – | Litze für Servo-Leistung | Querschnitt ≥ 0,5 mm² (AWG20), kurze Leitungen |
| – | Stiftleisten/JST/Dupont-Kabel | für Signale |
| – | Schrumpfschlauch/Lötmaterial | zur Zugentlastung |

## 3. Pinbelegung (Firmware-Standard)

Die Signal-Pins sind in `src/main.cpp` als Konstanten definiert:

| Funktion | ESP32-S3 Pin | Konstante in `main.cpp` |
|----------|--------------|--------------------------|
| Linker Servo (PWM-Signal)  | **GPIO4** | `PIN_SERVO_LEFT`  |
| Rechter Servo (PWM-Signal) | **GPIO5** | `PIN_SERVO_RIGHT` |

PWM-Parameter (LEDC): **50 Hz**, **16 Bit** Auflösung, Pulsbreite **500–2500 µs**
(→ in `main.cpp`: `SERVO_FREQ_HZ = 50`, `SERVO_RES_BITS = 16`).

> Pins ändern? Nur die beiden Konstanten `PIN_SERVO_LEFT` / `PIN_SERVO_RIGHT` in
> `src/main.cpp` anpassen und dabei die GPIO-Hinweise im Kopf der Datei beachten
> (keine Boot-Strapping-/Flash-/PSRAM-/UART0-/USB-Pins verwenden).

## 4. Servo-Anschluss (typische Farbkodierung)

| Servo-Leitung | Farbe (typisch) | Verbinden mit |
|---------------|-----------------|---------------|
| Signal        | weiß / gelb     | GPIO4 (links) bzw. GPIO5 (rechts) |
| Masse         | braun / schwarz | **GND des ESP32 UND Minus des DPS150** |
| Plus          | rot             | **Plus des DPS150 (7,0 V)** (nicht 3V3!) |

## 5. Servo-Daten & Stromversorgung (wichtig!)

### Verbauter Servo: Miuzei MS62-2-D-B

| Kenngröße | Wert |
|-----------|------|
| Typ | digitaler 25kg-Metallgetriebe-Servo |
| Drehbereich | **270°** |
| Betriebsspannung | **4,8–8,4 V** |
| Steuersignal | PWM, **500–2500 µs** (50–330 Hz) |
| Stellmoment | ~**27,5 kg·cm @ 8,4 V** |
| Stellgeschwindigkeit | ~**0,13 s/60° @ 8,4 V** |
| Abtrieb | **25T**-Verzahnung (Servohorn) |
| Schutz | wasserdicht (IP-Gehäuse) |

> Hinweis: Der **Stallstrom** wird vom Hersteller nicht öffentlich angegeben.
> Für diese Klasse (25kg-Digital-Servo) liegt er erfahrungsgemäß bei **2–3 A** pro Servo.

### Stromversorgung

- **Servos:** Labornetzteil **DPS150** auf **7,0 V (CV)** stellen, **Strombegrenzung (CC) ≈ 5 A**
  als Schutz gegen Kurzschluss/Blockieren. Betriebsspannungsbereich des Servos:
  **4,8–8,4 V** — nicht über 8,4 V betreiben. Für maximale Kraft auch 8,4 V möglich,
  7,0 V ist für den Probeaufbau ein guter Kompromiss.
- **Strombedarf:** Im Anlauf/unter Last kurzzeitig mehrere Ampere pro Servo
  (Stallstrom ca. 2–3 A). Daher die 5-A-Begrenzung am DPS150 setzen.
- **ESP32:** **NICHT mit den 7 V speisen!** Der ESP wird separat über **USB** versorgt.
  - `3V3`-Pin: max. 3,3 V → 7 V würden den ESP **sofort zerstören**.
  - `5V`-Pin: nominal 5 V (4,75–5,25 V). Der Regler auf dem DevKit regelt zwar auf
    3,3 V herunter, aber 7 V sind trotzdem **außerhalb der Spezifikation**: Der 5V-Pin
    liegt auf derselben Leitung wie USB-VBUS — bei gleichzeitig angestecktem USB drücken
    die 7 V in den PC zurück (Beschädigungsgefahr), und der Regler muss die größere
    Spannungsdifferenz als Wärme abführen. → ESP immer über **USB** (oder 5 V Buck auf den 5V-Pin) versorgen.
- **Bulk-Kondensator** (470–1000 µF) direkt an den Plus/Minus des DPS150-Ausgangs
  (so nah wie möglich an den Servos), um Spannungseinbrüche beim Anlauf abzufangen.
- **Gemeinsame Masse:** ESP32-GND und DPS150-Minus **müssen** verbunden sein,
  sonst ist das PWM-Signal nicht referenziert und die Servos zucken/laufen nicht.

## 6. Verdrahtung Schritt für Schritt

1. **Alles spannungsfrei** (USB ab, DPS150-Ausgang aus).
2. **Masse zuerst:** ESP32-GND mit DPS150-Minus verbinden.
3. **Signale:**
   - linker Servo → Signal an **GPIO4**
   - rechter Servo → Signal an **GPIO5**
4. **Servo-Plus:** beide roten Leitungen an **+ des DPS150 (7,0 V)**.
5. **Servo-Masse:** beide braunen/schwarzen Leitungen an **Minus des DPS150**
   (und damit an die gemeinsame Masse).
6. **Bulk-Kondensator** polaritätsrichtig zwischen Plus und Minus des DPS150-Ausgangs.
7. **DPS150 einstellen:** CV 7,0 V, CC 5 A — **vor** dem Anschließen der Last.
8. **Kontrolle:** keine Verbindung von Servo-Plus zum 3V3-/5V-Pin des Boards!
9. Erst jetzt: ESP32 per USB anschließen, dann DPS150-Ausgang einschalten.

## 7. Inbetriebnahme & Sicherheit

- **Erster Test ohne Gestänge:** Servos frei drehen lassen (nicht blockieren), damit
  sich nichts verklemmt, bevor die Kinematik kalibriert ist.
- **Spannungs-Reihenfolge:** zuerst GND verbinden, dann Spannung einschalten;
  beim Umbauen zuerst DPS150-Ausgang aus, dann USB.
- **Logikpegel:** 3,3 V-Signal reicht bei den meisten Digital-Servos. Falls ein Servo
  unzuverlässig reagiert, optional einen 3,3 V → 5 V-Levelshifter in die Signalleitung.
- **Kalibrierung:** Vor dem Aufsetzen der Kurbeln/Servohörner die Nullpunkte in
  `src/main.cpp` (`servos.setCalibrationLeft(...)` / `setCalibrationRight(...)`)
  ermitteln und eintragen — sonst kann die Startposition (0, 170) falsch angefahren
  werden und das Gestänge anschlagen.
- **Zugentlastung:** Signal-/Leistungsleitungen fixieren, damit beim Bewegen keine
  Stecker abrutschen oder Kurzschlüsse entstehen.

## 8. Nächste Schritte (Checkliste Probeaufbau)

- [x] DPS150 auf CV 7,0 V / CC 5 A eingestellt (vor dem Anschließen)
- [x] Gemeinsame Masse (ESP32-GND ↔ DPS150-Minus) hergestellt
- [x] Signal auf GPIO4 (links) / GPIO5 (rechts) geprüft
- [x] Servos ohne Gestänge an `pio run -e esp32s3 -t upload` getestet
- [x] Nullpunkt-/Richtungs-Kalibrierung in `src/main.cpp` gesetzt (am Modell verifiziert)
- [x] Gestänge montiert, Bewegung der Demo-Punkte (versteckt / Links-/Rechts-/Oben-Peek) beobachtet

## 9. Ausblick: Realbetrieb (ESP über Step-Down)

Im **Probeaufbau** wird der ESP über **USB** versorgt. Für den **Realbetrieb** ohne PC
kann der ESP aus demselben 7V-Ausgang des DPS150 gespeist werden — aber **nur über einen
5V-Step-Down (Buck)**:

```
DPS150 @ 7,0 V
   ├──► Servos (Plus, rot)
   └──► 5V-Buck (Step-Down) ──► ESP32 5V-Pin
                                  ESP32 GND ──► DPS150 Minus (gemeinsame Masse)
```

- **Buck-Ausgang:** exakt **5,0 V** einstellen, an den **5V-Pin** des ESP32 (nicht 3V3!).
- **Gemeinsame Masse:** Buck-Minus mit DPS150-Minus und ESP32-GND verbinden.
- **Nie gleichzeitig USB und Buck:** im Realbetrieb kein USB-Kabel anschließen, sonst
  speisen zwei Quellen parallel (Rückfluss-Gefahr). Zum Flashen USB verwenden (Buck trennen
  oder Buck-Ausgang aus).

## 10. Nächste Schritte (Checkliste Realbetrieb)

- [ ] 5V-Step-Down beschafft und auf exakt 5,0 V eingestellt (vor Anschluss messen)
- [ ] Buck-Ausgang an ESP32 **5V-Pin**, Buck-Minus an gemeinsame Masse
- [ ] Im Realbetrieb kein USB angeschlossen (nur zum Flashen)
- [ ] Spannungen mit Multimeter geprüft: 7,0 V (Servos), 5,0 V (ESP), gemeinsame Masse

## 11. LiPo-Betrieb (aktueller Stand)

Der Aufbau läuft inzwischen mit **LiPo** statt DPS150. Wichtig dabei:

- **Versorgung:** 2S-LiPo (voll ~8,4 V, leer ~6,0–6,6 V). Servos direkt am LiPo (4,8–8,4 V),
  ESP32 über einen **5V-Buck (LM2596)** am 5V-Pin.
- **Spannungsreserve:** Bei ~7 V am LM2596 ist der Dropout knapp; Servo-Lastspitzen können die
  5V-Schiene kurz einbrechen lassen → **Brownout**. Abhilfe:
  - Stützkondensatoren am Buck-Eingang und -Ausgang (470–1000 µF),
  - **Servo-Anfahrrampe** begrenzt die Stromspitzen (in `main.cpp`, Befehl `RAMP`).
- **Gemeinsame Masse** bleibt zwingend (LiPo-Minus ↔ ESP32-GND).
- **Nie gleichzeitig USB und Buck/LiPo** betreiben.

