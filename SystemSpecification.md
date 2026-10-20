# Systemspezifikation: 2D-planares 5-Gelenk-Zielsystem (Inverse Kinematik)

## 1. Projektübersicht
Entwicklung einer C++-Firmware (ESP32, PlatformIO) zur Steuerung eines dynamischen Hostage-Target-Systems (Geiselziel) auf Basis eines symmetrischen, ebenen 5-Gelenk-Parallelgetriebes (Fünfgelenkgetriebe) mit 2 Freiheitsgraden (2-DOF).
Der Nutzer bzw. das System gibt die gewünschten kartesischen Koordinaten $(X, Y)$ des zentralen Gelenks (End-Effektor) vor; die Firmware berechnet daraus in Echtzeit die benötigten Servowinkel $(\theta_L, \theta_R)$ mittels inverser Kinematik (IK).

### Mechanik-Übersicht
- **2 Servos** an der Basis ($S_L$, $S_R$) treiben je eine **Kurbel** (Kurbelarm, Länge $L_1$) an.
- Jede Kurbel ist mit einer **Schubstange** (Länge $L_2$) verbunden.
- Beide Schubstangen treffen sich im **zentralen Gelenk** $P(X, Y)$ – dem End-Effektor.
- Am selben Punkt ist mittig eine **Querstange** drehbar gelagert; zwei Gummibänder (links und rechts des Lagers, je eines pro Schubstange) halten diese Querstange **senkrecht zur Winkelhalbierenden** der Schubstangen. Auf der Querstange ist **nach oben** das Pappziel montiert (siehe §3).

### Skizze (Prinzip)
```
                        +-------------+
                        |    ZIEL     |   <- Pappziel (Hostage-Target),
                        |  (Target)   |      nach oben auf der Querstange
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
- **Hinweis zu den Maßen:** Alle mechanischen Maße (`D`, `L1`, `L2`, …) werden im späteren Code zentral als leicht anpassbare Konstanten geführt, damit Änderungen nur an einer Stelle nötig sind.

## 3. Mathematisches Modell (Inverse Kinematik)
Für eine vorgegebene Zielkoordinate $P(X, Y)$ des zentralen Gelenks:

### Berechnung linker Arm:
1. Vektor von $S_L$ nach $P$:
   $$\Delta X_L = X - (-D/2)$$
   $$\Delta Y_L = Y$$
2. Abstand von $S_L$ nach $P$:
   $$d_L = \sqrt{\Delta X_L^2 + \Delta Y_L^2}$$
3. Basiswinkel des Vektors $S_L \to P$:
   $$\alpha_L = \text{atan2}(\Delta Y_L, \Delta X_L)$$
4. Innenwinkel des Dreiecks (Kosinussatz):
   $$\cos(\beta_L) = \frac{L_1^2 + d_L^2 - L_2^2}{2 \cdot L_1 \cdot d_L}$$
   $$\beta_L = \text{acos}(\cos(\beta_L))$$
5. Resultierender Kurbelwinkel links $\theta_L$ (gegen den Uhrzeigersinn, von der positiven X-Achse aus gemessen):
   - **Konfiguration „Knie nach außen" (Standard):**
     $$\theta_L = \alpha_L + \beta_L$$

### Berechnung rechter Arm:
1. Vektor von $S_R$ nach $P$:
   $$\Delta X_R = X - (D/2)$$
   $$\Delta Y_R = Y$$
2. Abstand von $S_R$ nach $P$:
   $$d_R = \sqrt{\Delta X_R^2 + \Delta Y_R^2}$$
3. Basiswinkel des Vektors $S_R \to P$:
   $$\alpha_R = \text{atan2}(\Delta Y_R, \Delta X_R)$$
4. Innenwinkel:
   $$\cos(\beta_R) = \frac{L_1^2 + d_R^2 - L_2^2}{2 \cdot L_1 \cdot d_R}$$
   $$\beta_R = \text{acos}(\cos(\beta_R))$$
5. Resultierender Kurbelwinkel rechts $\theta_R$ (gegen den Uhrzeigersinn, von der positiven X-Achse aus gemessen):
   - **Konfiguration „Knie nach außen" (Standard):**
     $$\theta_R = \alpha_R - \beta_R$$

### Ausrichtung des Zielkopfes (elastische Kopplung über Querstange):
Am zentralen Gelenk $P(X, Y)$ (hier treffen die beiden Schubstangen zusammen) ist mittig eine weitere **Querstange** drehbar gelagert. Links und rechts des Lagers sind zwei schwarze Gummibänder befestigt – eines zur linken Schubstange ($L_{2,L}$) und eines zur rechten Schubstange ($L_{2,R}$) –, damit die Querstange nicht frei dreht. Diese elastische Kopplung hält die Querstange stets **im rechten Winkel (senkrecht) zur Winkelhalbierenden** der beiden Schubstangen: Steht die Winkelhalbierende senkrecht, liegt die Querstange waagerecht. Auf dieser Querstange ist **nach oben** das Pappziel montiert; das Ziel zeigt dadurch immer **entlang der Winkelhalbierenden** (senkrecht zur Querstange).
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
5. Resultierender Winkel der Ziel-Zentrallinie $\psi$ (Pappziel nach oben auf der Querstange montiert, also senkrecht zur Querstange und damit entlang der Winkelhalbierenden):
   $$\psi = \theta_q - 90^\circ = \phi = \frac{\gamma_L + \gamma_R}{2}$$
   *(Bei $X = 0$ ist $\psi = 90^\circ$ ($\pi/2$ rad), das Ziel steht also senkrecht/aufrecht. Wandert $P$ seitlich, kippt die Winkelhalbierende und damit das Ziel in die Peek-Richtung.)*

## 4. Sicherheit, Singularitäten & Arbeitsraum-Grenzen (KRITISCH)
Der IK-Löser (`bool calculateIK(float x, float y, float &thetaL, float &thetaR)`) MUSS unerreichbare oder gefährliche Koordinaten zurückweisen und `false` liefern, bevor die Servos bewegt werden:
1. **Dreiecksungleichung / Reichweitenlimit:**
   $$\vert{}L_1 - L_2\vert{} + \epsilon < d_{L,R} < (L_1 + L_2) - \epsilon$$
   (Mit Sicherheitsabstand $\epsilon \approx 5\text{ mm}$, um Totpunkt-Singularitäten / volles Ausstrecken und Verklemmen zu vermeiden).
2. **Kosinus-Domänen-Clamping:** Argumente für `acos()` strikt in $[-1.0, 1.0]$ halten.
3. **Mechanische Kollision & Servo-Grenzen:**
   - Sicherstellen, dass $\theta_L$ und $\theta_R$ innerhalb der physischen Servo-Stellgrenzen bleiben und nicht mit der Grundplatte oder dem benachbarten Servokörper kollidieren.
   - Eine sichere Bounding-Box / Arbeitsraum-Polygon für $(X, Y)$ definieren, in dem das Gestänge in Parkstellung hinter dem zentralen Pfosten verborgen bleibt.

## 5. Anforderungen an die Software-Architektur
1. **Klasse `Kinematics`:**
   - Reine Mathematikklasse, hardwareunabhängig.
   - Wandelt $(X, Y) \to (\theta_L, \theta_R)$ in Radiant/Grad um.
2. **Klasse `ServoController`:**
   - Bildet kinematische Winkel $(\theta_L, \theta_R)$ auf PWM-Mikrosekunden (`500`–`2500` µs) ab.
   - Muss Kalibrier-Offsets (`zeroOffsetLeft`, `zeroOffsetRight`) und Richtungs-Invertierungs-Flags (`invertLeft`, `invertRight`) enthalten, um Montagetoleranzen der Servohörner auszugleichen.
3. **`MotionPlanner` / Interpolation:**
   - Nicht-blockierende Trajektoriengenerierung.
   - Zwei Bewegungstypen:
     - **Snap-/Pop-out-Bewegung:** Direkter Sprung zum Ziel $(X, Y)$ mit maximaler physischer Servogeschwindigkeit.
     - **Lineare/sanfte Bewegung (Sweep/Joystick):** Interpoliert zwischen aktuellem $(X_0, Y_0)$ und Ziel $(X_1, Y_1)$ in kleinen Zeitschritten (z.B. 50Hz/20ms Update-Schleife) mit linearer Interpolation oder S-Kurve/Kosinus-Easing; dabei wird in jedem Zwischenschritt der IK-Löser aufgerufen, sodass sich der Kopf auf einer kontrollierten Bahn bewegt.
4. **Betriebsmodi:**
   - `MANUAL`: Direkte X/Y-Eingabe (z.B. Joystick/Web-UI/BLE).
   - `RECORD_PLAYBACK`: Ringpuffer/Array zur Aufzeichnung von $(X, Y)$-Koordinaten in festen Zeitabständen (z.B. 20Hz) und deren Wiedergabe als Schleife.
   - `RANDOM`: Zustandsmaschine, die zufällig gültige Presets wählt (Versteckt, Links Peek, Rechts Peek, Oben Peek) mit zufälligen Halte- und Versteck-Dauern.
   - `PRESET`: Fest kodierte Trajektorien (Sweep links–rechts, Pop-out und zurück).


