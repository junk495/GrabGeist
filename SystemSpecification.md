# Systemspezifikation: GrabGeist — 2D-planares 5-Gelenk-Getriebe (Inverse Kinematik)

## 1. Projektübersicht
Entwicklung einer C++-Firmware (ESP32, PlatformIO) zur Steuerung eines Geists (GrabGeist), der hinter einem Grabstein immer wieder hervorschaut, auf Basis eines symmetrischen, ebenen 5-Gelenk-Parallelgetriebes (Fünfgelenkgetriebe) mit 2 Freiheitsgraden (2-DOF).
Der Nutzer bzw. das System gibt die gewünschten kartesischen Koordinaten $(X, Y)$ des zentralen Gelenks (End-Effektor) vor; die Firmware berechnet daraus in Echtzeit die benötigten Servowinkel $(\theta_L, \theta_R)$ mittels inverser Kinematik (IK).

### Mechanik-Übersicht
- **2 Servos** an der Basis ($S_L$, $S_R$) treiben je eine **Kurbel** (Kurbelarm, Länge $L_1$) an.
- Jede Kurbel ist mit einer **Schubstange** (Länge $L_2$) verbunden.
- Beide Schubstangen treffen sich im **zentralen Gelenk** $P(X, Y)$ – dem End-Effektor.
- Am selben Punkt ist mittig eine **Querstange** drehbar gelagert; zwei Gummibänder (links und rechts des Lagers, je eines pro Schubstange) halten diese Querstange **senkrecht zur Winkelhalbierenden** der Schubstangen. Auf der Querstange ist **nach oben** der Geist montiert (siehe §3).

### Skizze (Prinzip)
```
                        +-------------+
                        |    GEIST    |   <- Geist (Ghost),
                        |   (Ghost)   |      nach oben auf der Querstange
                        +------+------+
                               |
                               |   psi = phi  (Ziel entlang der
                               |              Winkelhalbierenden)
                               |
                     ==========+==========   <- Querstange (senkrecht zu phi)
                               |
                             P(X,Y)    <- zentrales Gelenk (End-Effektor)
                            /      \
                           /        \
        Schubstange L2,L  /          \   Schubstange L2,R
                (125 mm)  /            \        (125 mm)
                         /              \
                      E_L                E_R   <- Ellbogen (Kurbelenden)
                       |                  |
          Kurbel L1    |                  |    Kurbel L1
           (90 mm)     |                  |     (90 mm)
                       |                  |
                     S_L                 S_R    <- Servoachsen
                 (-D/2, 0)           (+D/2, 0)
                 +------------- D = 65 mm ------------+

   Koordinatensystem: X -> rechts, Y -> oben, Ursprung (0,0) = Mitte zwischen S_L und S_R.
   (Prinzip-Skizze: Querstange senkrecht zur Winkelhalbierenden, Ziel entlang der Winkelhalbierenden — siehe §3.)
```

## 2. Koordinatensystem & mechanische Parameter
- **Ursprung $(0,0)$:** Mittelpunkt zwischen den beiden Servo-Drehachsen.
- **X-Achse:** horizontal (positiv = rechts, negativ = links).
- **Y-Achse:** vertikal (positiv = nach oben, von den Servos weg).
- **Kinematische Konstanten (im Code als zentral anpassbare Variablen / `constexpr float`, Einheit mm):**
  - `D = 65.0f`: Abstand zwischen linkem und rechtem Servo.
  - `L1 = 90.0f`: Kurbellänge (Kurbelarm, am Servohorn befestigt) [ggf. nach CAD anpassen].
  - `L2 = 125.0f`: Schubstangenlänge (von der Kurbel zum zentralen Gelenk) [ggf. nach CAD anpassen].
- **Servopositionen:**
  - Linker Servo $S_L = (-D/2, 0) = (-32.5, 0)$
  - Rechter Servo $S_R = (+D/2, 0) = (+32.5, 0)$
- **Aktoren:** 2× 25kg Digital-RC-Servos (270° Stellbereich, PWM 50Hz–330Hz, Impulsbreite 500µs–2500µs).
- **Servo-Winkelgrenzen** (aus Kalibrierung abgeleitet, am Modell verifiziert):
  - $\theta_L \in [27.9°, 297.9°]$ ,  $\theta_R \in [-113.85°, 156.15°]$
- **Kollisions-Parameter** (am Modell ERPROBEN):
  - Zentrale Sperrzone (Servogehäuse): $X \in [-45, +45]$ mm , $Y \in [-60, +20]$ mm
  - Mindestabstand zur gegenüberliegenden Servoachse: 18 mm
  - Mindest-Ellbogen-Abstand (608ZZ-Gehäuse): 30 mm
- **Hinweis zu den Maßen:** Alle mechanischen Maße (`D`, `L1`, `L2`, …) werden im späteren Code zentral als leicht anpassbare Konstanten geführt, damit Änderungen nur an einer Stelle nötig sind.

## 3. Mathematisches Modell (Inverse Kinematik) — bimodal
Für eine vorgegebene Zielkoordinate $P(X, Y)$ des zentralen Gelenks hat **jeder Arm zwei Lösungen**.

Für einen Arm (Servo bei $S_{L,R}$; $\Delta X = X \mp D/2$ , $\Delta Y = Y$):
1. Abstand:  $$d = \sqrt{\Delta X^2 + \Delta Y^2}$$
2. Basiswinkel:  $$\alpha = \text{atan2}(\Delta Y, \Delta X)$$
3. Innenwinkel (Kosinussatz):  $$\beta = \text{acos}\!\left(\frac{L_1^2 + d^2 - L_2^2}{2 \cdot L_1 \cdot d}\right)$$
4. Zwei Kandidaten je Arm:
   - **Linker Arm:**  $\theta_{L,1} = \alpha_L + \beta_L$ („Knie nach außen") ,  $\theta_{L,2} = \alpha_L - \beta_L$ („Over-Center")
   - **Rechter Arm:**  $\theta_{R,1} = \alpha_R - \beta_R$ („Knie nach außen") ,  $\theta_{R,2} = \alpha_R + \beta_R$ („Over-Center")

Aus den 2×2 = **4 Kandidaten-Paaren** wird das erste **kollisionsfreie** Paar gewählt (§4):
- Standard: „Knie nach außen" bevorzugt.
- Mit Hysterese-Hint: das dem **aktuellen Servowinkel** nächste Paar (vermeidet Konfigurationssprünge).

Die Over-Center-Lösung ($\theta > 180°$ bzw. negativ) erschließt die unteren seitlichen „Taschen" (negatives $Y$) → **nierenförmiger** Arbeitsraum.

### Ausrichtung des Geisterkopfs (elastische Kopplung über Querstange):
Am zentralen Gelenk $P(X, Y)$ (hier treffen die beiden Schubstangen zusammen) ist mittig eine weitere **Querstange** drehbar gelagert. Links und rechts des Lagers sind zwei schwarze Gummibänder befestigt – eines zur linken Schubstange ($L_{2,L}$) und eines zur rechten Schubstange ($L_{2,R}$) –, damit die Querstange nicht frei dreht. Diese elastische Kopplung hält die Querstange stets **im rechten Winkel (senkrecht) zur Winkelhalbierenden** der beiden Schubstangen: Steht die Winkelhalbierende senkrecht, liegt die Querstange waagerecht. Auf dieser Querstange ist **nach oben** der Geist montiert; er zeigt dadurch immer **entlang der Winkelhalbierenden** (senkrecht zur Querstange).
1. Positionen der Ellbogen (Knie) links und rechts:
   $$E_L = (-D/2 + L_1 \cos(\theta_L), \, L_1 \sin(\theta_L))$$
   $$E_R = (+D/2 + L_1 \cos(\theta_R), \, L_1 \sin(\theta_R))$$
2. Winkel der Schubstangen, die im zentralen Gelenk $P(X, Y)$ ankommen:
   $$\gamma_L = \text{atan2}(Y - E_{L,y}, \, X - E_{L,x})$$
   $$\gamma_R = \text{atan2}(Y - E_{R,y}, \, X - E_{R,x})$$
3. Winkelhalbierende der beiden Schubstangen (ihr „halber Winkel"):
   $$\phi = \frac{\gamma_L + \gamma_R}{2}$$
4. Winkel der Querstange $\theta_q$ (senkrecht zur Winkelhalbierenden):
   $$\theta_q = \phi + 90^\circ$$
   *(Bei $X = 0$ ist $\phi = 90^\circ$, die Querstange liegt also waagerecht.)*
5. Resultierender Winkel der Geist-Zentrallinie $\psi$ (Geist nach oben auf der Querstange montiert, also senkrecht zur Querstange und damit entlang der Winkelhalbierenden):
   $$\psi = \theta_q - 90^\circ = \phi = \frac{\gamma_L + \gamma_R}{2}$$
   *(Bei $X = 0$ ist $\psi = 90^\circ$ ($\pi/2$ rad), der Geist steht also senkrecht/aufrecht. Wandert $P$ seitlich, kippt die Winkelhalbierende und damit der Geist in die Peek-Richtung.)*

## 4. Sicherheit, Singularitäten & Arbeitsraum (KRITISCH)
Der IK-Löser (`bool calculateIK(...)`) MUSS ungültige/kollidierende Koordinaten mit `false` ablehnen, bevor die Servos bewegt werden. Die Gültigkeit ergibt sich aus **7 Prüfungen**:

1. **Dreiecksungleichung / Reichweite:**
   $$\vert{}L_1 - L_2\vert{} + \epsilon < d_{L,R} < (L_1 + L_2) - \epsilon$$
   (Sicherheitsabstand $\epsilon \approx 5\text{ mm}$ → $40\ldots 210$ mm).
2. **Kosinus-Domänen-Clamping:** Argumente für `acos()` strikt in $[-1.0, 1.0]$ halten.
3. **Servo-Winkelgrenzen:**  $\theta_L \in [27.9°, 297.9°]$ ,  $\theta_R \in [-113.85°, 156.15°]$  (aus Kalibrierung, am Modell verifiziert).
4. **Zentrale Sperrzone (Servogehäuse):**  $P$ darf nicht in $X \in [-45, +45]$ , $Y \in [-60, +20]$ liegen.
5. **Link↔Achsen-Clearance:**  Kurbeln/Schubstangen halten ≥ 18 mm Abstand zur gegenüberliegenden Servoachse.
6. **Kein Kreuz/Verklemmen:**  Kurbeln kreuzen sich nicht; Ellbogen-Abstand ≥ 30 mm; Schubstangen kreuzen keine Kurbel.
7. **Gelenk-Orientierung (kein Umklappen):**
   $$(E_{R,x} - E_{L,x})(Y - E_{L,y}) - (E_{R,y} - E_{L,y})(X - E_{L,x}) > 0$$

### Arbeitsraum (nierenförmig) & Karte
Aus den 7 Prüfungen ergibt sich ein **nierenförmiger** Arbeitsraum: große obere Kalotte (positives $Y$) + zwei untere seitliche Taschen (negatives $Y$, über Over-Center), Mitte unten gesperrt (Servogehäuse).

- **Karte (`Workspace`):** Beim Start wird ein Raster ($X \in [-220, +220]$ , $Y \in [-150, +215]$ , Schritt 5 mm) gesampelt; alle gültigen $(X, Y)$ werden als Bitmap abgelegt.
- **Snapping:** Tippt der Nutzer auf einen ungültigen Punkt, wird der **nächste gültige** Punkt gewählt.
- **Pfadplanung:** Da der Arbeitsraum nicht-konvex ist, kreuzt eine gerade Linie (z.B. links↔rechts) die Sperrzone → automatischer **Umweg per A***.

## 5. Anforderungen an die Software-Architektur
1. **Klasse `Kinematics`:** (hardwarefrei, pure Mathematik)
   - Bimodale IK: $(X, Y) \to (\theta_L, \theta_R)$ mit 4 Kandidaten + 7 Kollisionsfiltern + Hysterese.
   - `bool calculateIK(...)` und `bool isReachable(x, y)`.
2. **Klasse `Workspace`:** (hardwarefrei)
   - Raster-Karte des Arbeitsraums (`generate()`), `isValid()`, `nearestValid()` (Snap), `findPath()` (A*).
3. **Klasse `ServoController`:**
   - Bildet kinematische Winkel $(\theta_L, \theta_R)$ auf PWM-Mikrosekunden (`500`–`2500` µs) ab.
   - Muss Kalibrier-Offsets (`zeroOffsetLeft`, `zeroOffsetRight`) und Richtungs-Invertierungs-Flags (`invertLeft`, `invertRight`) enthalten, um Montagetoleranzen der Servohörner auszugleichen.
4. **Bahnsteuerung (Wegpunkt-Folge + Servo-Rampe):**
   - **Hindernisbewusst:** fährt die A*-Wegpunkte des `Workspace` ab (Umweg um die Sperrzone) und reicht den aktuellen Winkel als Hysterese-Hint an die IK durch.
   - **`ServoSpeed`-Rampe:** deckelt die maximale PWM-Änderungsrate (µs/s) und begrenzt damit Beschleunigung/Stromspitzen; `SPEED %` skaliert darunter.
   - Zwei Bewegungstypen:
     - **Snap-/Pop-out-Bewegung:** Direkter Sprung zum Ziel $(X, Y)$ mit maximaler physischer Servogeschwindigkeit.
     - **Lineare/sanfte Bewegung (Sweep):** Wegpunkt-Folge mit begrenzter Geschwindigkeit; die Rampe sorgt für eine kontrollierte Bahn.
5. **Klasse `CommandParser`:** (hardwarefrei)
   - Zerlegt Textbefehle (`MOVE`, `PEEK`, `STOP`, `STATUS`, `RAW`, `AUTO`, `RANDOM`, `REC`, `PLAY`, `MAP`, `RAMP`, `SPEED`, `WIFI`, `RESETLOG`) in einen strukturierten `Command`.
6. **Betriebsmodi:**
   - `MANUAL`: Direkte X/Y-Eingabe (Tap-to-Move-Karte / Joystick / BLE).
   - `RECORD_PLAYBACK`: Ringpuffer/Array zur Aufzeichnung von $(X, Y)$-Koordinaten in festen Zeitabständen (z.B. 20Hz) und deren Wiedergabe als Schleife.
   - `RANDOM`: Zustandsmaschine, die zufällig gültige Presets wählt (Versteckt, Links Peek, Rechts Peek, Oben Peek) mit zufälligen Halte- und Versteck-Dauern.
   - `PRESET`: Fest kodierte Trajektorien (Sweep links–rechts, Pop-out und zurück).
      → Umsetzung: ersetzt durch **JSON-Abläufe** (`Sequence`, über die Web-UI/JSON-API bearbeitet und per `POST /api/run` abgespielt).

> **Implementierungsstatus:** `MANUAL`, `RECORD_PLAYBACK` (Pulsbreiten statt (X,Y), da „Versteckt"/„Beide oben" direkte Servo-Positionen sind) und `RANDOM` sind umgesetzt. Zusätzlich existieren eine konfigurierbare Servo-Rampe (`ServoSpeed`), ein Reset-Log (NVS), ein WiFi-SoftAP mit JSON-API (`ApiServer`) und ein persistent gespeicherter JSON-Ablauf (`Sequence`). `PRESET` ist durch JSON-Abläufe ersetzt.



