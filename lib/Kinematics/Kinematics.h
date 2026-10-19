// Kinematics.h
// Inverse Kinematik fuer das 5-Gelenk-Zielsystem (HostageTarget).
// Reine Mathematik - hardwareunabhaengig. Einheit: mm, Winkel in Radiant.

#pragma once

#include <cmath>

class Kinematics {
public:
    // ------------------------------------------------------------------
    // Mechanische Parameter (mm) - zentral & leicht anpassbar.
    // Bei CAD-Aenderungen NUR hier anpassen.
    // ------------------------------------------------------------------
    static constexpr float D       = 65.0f;   // Abstand der beiden Servoachsen
    static constexpr float L1      = 90.0f;   // Kurbellaenge (Kurbelarm)
    static constexpr float L2      = 120.0f;  // Schubstangenlaenge
    static constexpr float EPSILON = 5.0f;    // Sicherheitsabstand (mm)

    static constexpr float kPi = 3.14159265358979323846f;

    // Grad <-> Radiant
    static constexpr float degToRad(float deg) { return deg * kPi / 180.0f; }
    static constexpr float radToDeg(float rad) { return rad * 180.0f / kPi; }

    // ------------------------------------------------------------------
    // Inverse Kinematik (Kernfunktion).
    // Zielposition P(x, y) [mm] -> Servowinkel thetaL, thetaR [rad].
    // Liefert false, wenn die Position unerreichbar/ungueltig ist.
    // ------------------------------------------------------------------
    static bool calculateIK(float x, float y, float& thetaL, float& thetaR);

    // Vorwaerts-Kinematik (fuer Tests/Debug):
    // Servowinkel -> Position P(x, y). false, wenn die Schubstangen sich nicht treffen.
    static bool forwardKinematics(float thetaL, float thetaR, float& x, float& y);

    // Winkel der Ziel-Zentrallinie psi [rad].
    // (Das Ziel steht senkrecht auf der Querstange -> es zeigt entlang der
    //  Winkelhalbierenden der beiden Schubstangen.)
    static bool calculateTargetCenterlineAngle(float x, float y, float& psi);

private:
    // IK fuer einen einzelnen Arm (servoX = x-Koordinate der Servoachse).
    static bool solveArm(float x, float y, float servoX, bool isLeft, float& theta);
};
