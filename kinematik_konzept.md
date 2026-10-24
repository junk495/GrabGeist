# Kinematik-Konzept: 5-Gelenk-Getriebe (Inverse Kinematik)

## Kernidee
Ein Geist (GrabGeist) wird über ein ebenes 5-Gelenk-Parallelgetriebe mit **2 Servos** bewegt.
Der Nutzer steuert **nicht** die einzelnen Servos, sondern die **resultierende Zielposition P(X, Y)**.
Die Software berechnet daraus die benötigten Servowinkel (Inverse Kinematik, IK).

## Mechanischer Aufbau
- 2 Servos an der Basis, Achsabstand **D = 65 mm**.
- Jeder Servo treibt eine **Kurbel** (Länge **L1 = 90 mm**) an.
- An jeder Kurbel hängt eine **Schubstange** (Länge **L2 = 125 mm**).
- Beide Schubstangen treffen sich im **zentralen Gelenk P(X, Y)** (End-Effektor).
- Am selben Punkt P ist eine **Querstange** drehbar gelagert (über 2 Gummibänder senkrecht zur
  Winkelhalbierenden der Schubstangen gehalten); darauf steht **nach oben** der Geist.

> Alle Maße als **leicht anpassbare Konstanten** (`constexpr float`, Einheit mm) führen.

## Koordinatensystem
- Ursprung (0,0): Mitte zwischen den beiden Servoachsen.
- X: horizontal (rechts positiv).  Y: vertikal (oben positiv, von den Servos weg).
- Servopositionen:  **S_L = (−D/2, 0)** ,  **S_R = (+D/2, 0)**.

## Inverse Kinematik (Zielposition → Servowinkel) — bimodal
Winkel werden **gegen den Uhrzeigersinn von der positiven X-Achse** gemessen; intern in **Radiant**.

Für jeden Arm gibt es **zwei** Lösungen (Kandidaten):
- Linker Arm:   θ_L,1 = α_L + β_L  („Knie nach außen") ,  θ_L,2 = α_L − β_L  („Over-Center")
- Rechter Arm:  θ_R,1 = α_R − β_R  („Knie nach außen") ,  θ_R,2 = α_R + β_R  („Over-Center")

mit  d = √(ΔX² + ΔY²) ,  α = atan2(ΔY, ΔX) ,  β = acos( (L1² + d² − L2²) / (2 · L1 · d) ).

Aus den 2×2 = **4 Kandidaten-Paaren** wird ein **kollisionsfreies** Paar gewählt:
- Standard: „Knie nach außen" bevorzugt.
- Mit Hysterese-Hint: das dem **aktuellen Servowinkel** nächste Paar (vermeidet Konfigurationssprünge).

Die Over-Center-Lösung (>180° bzw. negativ) erschließt die **unteren seitlichen Taschen**
(negatives Y) — dadurch entsteht der **nierenförmige** Arbeitsraum.

## Gültigkeitsprüfung & Kollisionsfilter (MUSS vor jeder Bewegung erfolgen)
Der IK-Solver (`bool calculateIK(...)`) muss ungültige/kollidierende Koordinaten mit `false` ablehnen:
1. **Reichweite:**  |L1 − L2| + ε < d < (L1 + L2) − ε ,  mit ε ≈ 5 mm  (→ 40…210 mm)
2. **acos-Domäne:** Argument vor `acos()` strikt auf [−1, 1] clampen.
3. **Servo-Grenzen:**  θ_L ∈ [27.9°, 297.9°] ,  θ_R ∈ [−113.85°, 156.15°]  (aus Kalibrierung, am Modell verifiziert)
4. **Zentrale Sperrzone (Servogehäuse):** P darf nicht in [−45, +45] × [−60, +20] liegen.
5. **Link↔Achsen-Clearance:** Kurbeln/Schubstangen halten ≥ 18 mm Abstand zur gegenüberliegenden Servoachse.
6. **Kein Kreuz/Verklemmen:** Kurbeln kreuzen sich nicht; Ellbogen-Abstand ≥ 30 mm; Stangen kreuzen keine Kurbel.
7. **Gelenk-Orientierung:** P bleibt auf der positiven Seite von E_L→E_R (kein Umklappen des Geists).

## Geist-Ausrichtung (Kopf-Orientierung, optional)
- Ellbogen:  E_L = (−D/2 + L1·cos θ_L, L1·sin θ_L) ,  E_R = (+D/2 + L1·cos θ_R, L1·sin θ_R)
- Schubstangenwinkel:  γ_L = atan2(Y − E_L.y, X − E_L.x) ,  γ_R = atan2(Y − E_R.y, X − E_R.x)
- Winkelhalbierende:  φ = (γ_L + γ_R)/2
- Querstange (senkrecht zu φ):  θ_q = φ + 90°
- Geist-Zentrallinie (senkrecht auf Querstange → entlang φ):  **ψ = φ**
  (Bei X = 0 steht der Geist senkrecht; wandert P seitlich, kippt er in die Peek-Richtung.)

## Plausibilitäts-Beispiel (D = 65, L1 = 90, L2 = 125)
- Mitte:           P(0, 170)     → θ_L ≈ 123.1° , θ_R ≈ 56.9°   (Knie nach außen)
- Linke Tasche:    P(−120, −40)  → θ_L ≈ 288.8° , θ_R ≈ 142.3°  (Over-Center)
- Rechte Tasche:   P(+120, −40)  → θ_L ≈ 37.7°  , θ_R ≈ −108.8° (Over-Center)

## Software-Bausteine
1. **`Kinematics`** (hardwarefrei): bimodale IK + Kollisionsfilter + `isReachable(x, y)` + Hysterese.
2. **`Workspace`** (hardwarefrei): Raster-Karte der erreichbaren Punkte, `nearestValid()` (Snap), `findPath()` (A*).
3. **`ServoController`**: Servowinkel → PWM (500–2500 µs), mit Kalibrier-Offsets und Invertier-Flags.
4. **`ServoSpeed`**: Servo-Rampe / Strombegrenzung (µs/s) — deckelt die maximale PWM-Änderungsrate.
5. **`CommandParser`**: Textbefehle → strukturierter `Command`.

## Rahmen
- ESP32-S3, PlatformIO, Arduino-Framework, C++ (gnu++17).
- Wiederverwendbare Klassen in `lib/`, Sketch in `src/main.cpp`, Unit-Tests (Unity, nativ) in `test/`.
