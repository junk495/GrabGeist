// Kinematics.cpp

#include "Kinematics.h"

#include <cmath>

bool Kinematics::calculateIK(float x, float y, float& thetaL, float& thetaR) {
    // Linker Servo S_L = (-D/2, 0), rechter Servo S_R = (+D/2, 0)
    if (!solveArm(x, y, -D / 2.0f, true, thetaL)) {
        return false;
    }
    if (!solveArm(x, y, +D / 2.0f, false, thetaR)) {
        return false;
    }
    return true;
}

bool Kinematics::solveArm(float x, float y, float servoX, bool isLeft, float& theta) {
    const float dx = x - servoX;
    const float dy = y;
    const float d  = std::sqrt(dx * dx + dy * dy);

    // 1. Reichweiten-Check (Dreiecksungleichung + Sicherheitsabstand)
    const float minReach = std::fabs(L1 - L2) + EPSILON;
    const float maxReach = (L1 + L2) - EPSILON;
    if (d <= minReach || d >= maxReach) {
        return false;
    }

    // 2. Basiswinkel des Vektors Servo -> P
    const float alpha = std::atan2(dy, dx);

    // 3. Innenwinkel (Kosinussatz); Argument defensiv auf [-1, 1] clampen
    float cosBeta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cosBeta > 1.0f) {
        cosBeta = 1.0f;
    } else if (cosBeta < -1.0f) {
        cosBeta = -1.0f;
    }
    const float beta = std::acos(cosBeta);

    // 4. Kurbelwinkel - Konfiguration "Knie nach aussen"
    theta = isLeft ? (alpha + beta) : (alpha - beta);
    return true;
}

bool Kinematics::forwardKinematics(float thetaL, float thetaR, float& x, float& y) {
    // Ellbogen (Kurbelenden)
    const float elx = -D / 2.0f + L1 * std::cos(thetaL);
    const float ely = L1 * std::sin(thetaL);
    const float erx = +D / 2.0f + L1 * std::cos(thetaR);
    const float ery = L1 * std::sin(thetaR);

    const float dx = erx - elx;
    const float dy = ery - ely;
    const float d  = std::sqrt(dx * dx + dy * dy);

    // Die Schubstangen (Radius L2) koennen sich nicht treffen
    if (d > 2.0f * L2 || d < 1e-6f) {
        return false;
    }

    // Schnittpunkt zweier Kreise mit Radius L2 um E_L und E_R
    const float a = d / 2.0f;                     // halber Ellbogen-Abstand
    const float h = std::sqrt(L2 * L2 - a * a);   // Hoehe ueber der Verbindungslinie

    const float mx = elx + a * dx / d;            // Mittelpunkt zwischen den Ellbogen
    const float my = ely + a * dy / d;

    const float ix1 = mx - h * dy / d;            // zwei Kandidaten (senkrechter Versatz)
    const float iy1 = my + h * dx / d;
    const float ix2 = mx + h * dy / d;
    const float iy2 = my - h * dx / d;

    // Oberen Schnittpunkt waehlen (Ziel zeigt von den Servos weg, "Knie nach aussen")
    if (iy1 >= iy2) {
        x = ix1;
        y = iy1;
    } else {
        x = ix2;
        y = iy2;
    }
    return true;
}

bool Kinematics::calculateTargetCenterlineAngle(float x, float y, float& psi) {
    float thetaL, thetaR;
    if (!calculateIK(x, y, thetaL, thetaR)) {
        return false;
    }

    const float elx = -D / 2.0f + L1 * std::cos(thetaL);
    const float ely = L1 * std::sin(thetaL);
    const float erx = +D / 2.0f + L1 * std::cos(thetaR);
    const float ery = L1 * std::sin(thetaR);

    const float gammaL = std::atan2(y - ely, x - elx);
    const float gammaR = std::atan2(y - ery, x - erx);

    // Ziel-Zentrallinie = Winkelhalbierende der beiden Schubstangen
    psi = (gammaL + gammaR) / 2.0f;
    return true;
}
