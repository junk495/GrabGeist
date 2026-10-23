// MotionPlanner.h
// Nicht-blockierende Bahnplanung: interpoliert zwischen aktueller und
// Zielposition (X, Y) bzw. entlang einer Wegpunkt-Folge (A*-Pfad).
// Reine Mathematik - hardwareunabhaengig. Die inverse Kinematik wird NICHT hier,
// sondern im Sketch pro Zeitschritt auf die interpolierte Position angewendet.

#pragma once

#include <vector>

#include <Point.h>

class MotionPlanner {
public:
    enum class MoveType { SNAP, SMOOTH };
    enum class Easing   { LINEAR, COSINE };

    static constexpr float kPi = 3.14159265358979323846f;

    // Konfiguration
    void setDuration(float seconds);   // Dauer fuer moveTo (Einzelziel, SMOOTH)
    void setSpeed(float mmPerSec);     // Geschwindigkeit fuer moveAlongPath (SMOOTH) [mm/s]
    void setEasing(Easing easing);

    // Aktuelle Position setzen (ohne Bewegung, z.B. beim Start/Homing)
    void setPosition(float x, float y);

    // Bewegung starten: SNAP springt sofort, SMOOTH interpoliert weich.
    void moveTo(float x, float y, MoveType type);

    // Bewegung entlang einer Wegpunkt-Folge (z.B. A*-Pfad des Workspace).
    // SNAP springt zum letzten Punkt; SMOOTH interpoliert Segment fuer Segment.
    void moveAlongPath(const std::vector<Point>& waypoints, MoveType type);

    // Nicht-blockierender Update: dt = Sekunden seit dem letzten Aufruf.
    // Rueckgabe: true, solange eine SMOOTH-Bewegung noch laeuft.
    bool update(float dt);

    // Zustand
    float x() const;
    float y() const;
    bool isMoving() const;

private:
    float ease(float t) const;   // t in [0,1] -> [0,1]

    std::vector<Point> path_;     // Wegpunkte (inkl. Start)
    std::vector<float> segDuration_;  // Dauer je Segment [s]
    size_t seg_ = 0;              // aktuelles Segment path_[seg_] -> path_[seg_+1]
    float currentX_ = 0.0f;
    float currentY_ = 0.0f;
    float elapsed_ = 0.0f;
    float duration_ = 1.0f;       // Einzelziel-Dauer (moveTo)
    float speed_ = 150.0f;        // Bahngeschwindigkeit fuer Wegpunkt-Pfade [mm/s]
    Easing easing_ = Easing::COSINE;
    bool  moving_ = false;
};
