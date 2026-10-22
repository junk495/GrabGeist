// Kinematics.h
// Inverse Kinematik fuer das 5-Gelenk-Getriebe (GrabGeist).
// Reine Mathematik - hardwareunabhaengig. Einheit: mm, Winkel in Radiant.
// Bimodal (Knie nach aussen + Over-Center >180 Grad) mit Kollisions-Checks.

#pragma once

#include <cmath>

class Kinematics {
public:
    // ------------------------------------------------------------------
    // Mechanische Parameter (mm) - zentral & leicht anpassbar.
    // ------------------------------------------------------------------
    static constexpr float D       = 65.0f;   // Abstand der beiden Servoachsen
    static constexpr float L1      = 90.0f;   // Kurbellaenge (Kurbelarm)
    static constexpr float L2      = 125.0f;  // Schubstangenlaenge
    static constexpr float EPSILON = 5.0f;    // Sicherheitsabstand gegen Totpunkt (mm)

    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr float degToRad(float deg) { return deg * kPi / 180.0f; }
    static constexpr float radToDeg(float rad) { return rad * 180.0f / kPi; }

    // ------------------------------------------------------------------
    // Servo-Winkelgrenzen [rad] (aus der Kalibrierung abgeleitet,
    // am fertigen Modell VERIFIZIEREN!).
    //   links  : 500..2500 us  ->  theta =  27.90 .. 297.90 Grad
    //   rechts : 500..2500 us  ->  theta = -119.25 .. 150.75 Grad
    // ------------------------------------------------------------------
    static constexpr float THETA_L_MIN = 27.90f * kPi / 180.0f;
    static constexpr float THETA_L_MAX = 297.90f * kPi / 180.0f;
    static constexpr float THETA_R_MIN = -119.25f * kPi / 180.0f;
    static constexpr float THETA_R_MAX = 150.75f * kPi / 180.0f;

    // ------------------------------------------------------------------
    // Kollisions-Parameter (mm) - am Modell ERPROBEN!
    // ------------------------------------------------------------------
    static constexpr float SERVO_BOX_X_MIN = -45.0f;   // zentrale Sperrzone (Servogehaeuse)
    static constexpr float SERVO_BOX_X_MAX =  45.0f;
    static constexpr float SERVO_BOX_Y_MIN = -60.0f;
    static constexpr float SERVO_BOX_Y_MAX =  20.0f;
    static constexpr float R_CLEAR        =  18.0f;   // min. Abstand zur gegenueberliegenden Servoachse
    static constexpr float ELBOW_MIN_DIST =  30.0f;   // min. Ellbogen-Abstand (608ZZ-Gehaeuse)

    // ------------------------------------------------------------------
    // Inverse Kinematik (Kernfunktion).
    // Ohne Hint: bevorzugt "Knie nach aussen".
    // ------------------------------------------------------------------
    static bool calculateIK(float x, float y, float& thetaL, float& thetaR);

    // Mit Hysterese-Hint: bevorzugt die Konfiguration nahe den aktuellen Winkeln.
    static bool calculateIK(float x, float y, float hintThetaL, float hintThetaR,
                            float& thetaL, float& thetaR);

    // Nur Erreichbarkeit pruefen (ohne Winkel-Ausgabe).
    static bool isReachable(float x, float y);

    // Vorwaerts-Kinematik (fuer Tests/Debug, oberer Schnittpunkt).
    static bool forwardKinematics(float thetaL, float thetaR, float& x, float& y);

    // Winkel der Geist-Zentrallinie psi [rad].
    static bool calculateTargetCenterlineAngle(float x, float y, float& psi);

private:
    // Kandidaten eines Arms: thetaOut (Knie nach aussen), thetaOver (Over-Center).
    static bool solveArmCandidates(float x, float y, float servoX, bool isLeft,
                                   float& thetaOut, float& thetaOver);

    // Kollisions-Checks 2-5 (Check 1 = Servo-Grenzen, macht der Aufrufer).
    static bool collisionOk(float x, float y, float thetaL, float thetaR);
    static bool servoBoxOk(float x, float y);
    static bool clearanceOk(float x, float y, float elx, float ely, float erx, float ery);
    static bool crossingOk(float x, float y, float elx, float ely, float erx, float ery);
    static bool hingeOrientationOk(float x, float y, float elx, float ely, float erx, float ery);

    // Kern: Kandidaten berechnen, filtern, auswaehlen (mit/ohne Hint).
    static bool solveAndSelect(float x, float y, bool hasHint, float hintL, float hintR,
                               float& thetaL, float& thetaR);
};
