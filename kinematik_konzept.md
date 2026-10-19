# Kinematik-Konzept: 5-Gelenk-Zielsystem (Inverse Kinematik)

## Kernidee
Ein Übungsziel („Hostage Target") wird über ein ebenes 5-Gelenk-Parallelgetriebe mit **2 Servos** bewegt.
Der Nutzer steuert **nicht** die einzelnen Servos, sondern die **resultierende Zielposition P(X, Y)**.
Die Software berechnet daraus die benötigten Servowinkel (Inverse Kinematik, IK).

## Mechanischer Aufbau
- 2 Servos an der Basis, Achsabstand **D = 65 mm**.
- Jeder Servo treibt eine **Kurbel** (Länge **L1 = 90 mm**) an.
- An jeder Kurbel hängt eine **Schubstange** (Länge **L2 = 120 mm**).
- Beide Schubstangen treffen sich im **zentralen Gelenk P(X, Y)** (End-Effektor).
- Am selben Punkt P ist eine **Querstange** drehbar gelagert (über 2 Gummibänder senkrecht zur
  Winkelhalbierenden der Schubstangen gehalten); darauf steht **nach oben** das Pappziel.

> Alle Maße als **leicht anpassbare Konstanten** (`constexpr float`, Einheit mm) führen.

## Koordinatensystem
- Ursprung (0,0): Mitte zwischen den beiden Servoachsen.
- X: horizontal (rechts positiv).  Y: vertikal (oben positiv, von den Servos weg).
- Servopositionen:  **S_L = (−D/2, 0)** ,  **S_R = (+D/2, 0)**.

## Inverse Kinematik (Zielposition → Servowinkel)
Winkel werden **gegen den Uhrzeigersinn von der positiven X-Achse** gemessen; intern in **Radiant**.

### Linker Arm (S_L → P)
1. ΔX_L = X + D/2 ,  ΔY_L = Y
2. d_L = √(ΔX_L² + ΔY_L²)
3. α_L = atan2(ΔY_L, ΔX_L)
4. β_L = acos( (L1² + d_L² − L2²) / (2 · L1 · d_L) )
5. **θ_L = α_L + β_L**   ← Konfiguration „Knie nach außen" (Standard)

### Rechter Arm (S_R → P)
1. ΔX_R = X − D/2 ,  ΔY_R = Y
2. d_R = √(ΔX_R² + ΔY_R²)
3. α_R = atan2(ΔY_R, ΔX_R)
4. β_R = acos( (L1² + d_R² − L2²) / (2 · L1 · d_R) )
5. **θ_R = α_R − β_R**   ← Konfiguration „Knie nach außen" (Standard)

## Gültigkeitsprüfung (MUSS vor jeder Bewegung erfolgen)
Der IK-Solver (`bool calculateIK(x, y, &thetaL, &thetaR)`) muss ungültige Koordinaten mit `false` ablehnen:
1. **Reichweite (Dreiecksungleichung):**  |L1 − L2| + ε < d < (L1 + L2) − ε ,  mit ε ≈ 5 mm
2. **acos-Domäne:** Argument vor `acos()` strikt auf [−1, 1] clampen.
3. **Servo-Grenzen:** θ_L und θ_R innerhalb der physischen Stellgrenzen (270°-Servos) halten; Kollision vermeiden.

## Ziel-Ausrichtung (Kopf-Orientierung, optional)
- Ellbogen:  E_L = (−D/2 + L1·cos θ_L, L1·sin θ_L) ,  E_R = (+D/2 + L1·cos θ_R, L1·sin θ_R)
- Schubstangenwinkel:  γ_L = atan2(Y − E_L.y, X − E_L.x) ,  γ_R = atan2(Y − E_R.y, X − E_R.x)
- Winkelhalbierende:  φ = (γ_L + γ_R)/2
- Querstange (senkrecht zu φ):  θ_q = φ + 90°
- Ziel-Zentrallinie (senkrecht auf Querstange → entlang φ):  **ψ = φ**
  (Bei X = 0 steht das Ziel senkrecht; wandert P seitlich, kippt es in die Peek-Richtung.)

## Plausibilitäts-Beispiel (D = 65, L1 = 90, L2 = 120)
Ziel P(0, 170) mm → d ≈ 173.1 mm ,  θ_L ≈ 119.8° ,  θ_R ≈ 60.2°  (in rad: ≈ 2.090 / 1.051).
Beide Winkel liegen innerhalb der Reichweite → gültig.

## Software-Bausteine
1. **`Kinematics`** (hardwarefrei, pure Mathematik): `bool calculateIK(X, Y, &thetaL, &thetaR)`.
2. **`ServoController`**: Servowinkel → PWM (500–2500 µs), mit Kalibrier-Offsets (`zeroOffsetLeft`/`zeroOffsetRight`) und Invertier-Flags (`invertLeft`/`invertRight`).
3. **`MotionPlanner`**: nicht-blockierende Bahn (Sprung oder interpolierter Sweep; IK in jedem Zeitschritt).

## Rahmen
- ESP32-S3, PlatformIO, Arduino-Framework, C++ (gnu++17).
- Wiederverwendbare Klassen in `lib/`, Sketch in `src/main.cpp`, Unit-Tests (Unity, nativ) in `test/`.
