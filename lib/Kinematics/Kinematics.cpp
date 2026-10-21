// Kinematics.cpp

#include "Kinematics.h"

#include <cmath>

namespace {

constexpr float kTwoPi = 2.0f * Kinematics::kPi;

// Winkel in [minRad, minRad + 2*pi) normalisieren.
float normalizeAngle(float angle, float minRad) {
    float a = angle;
    while (a < minRad) {
        a += kTwoPi;
    }
    while (a >= minRad + kTwoPi) {
        a -= kTwoPi;
    }
    return a;
}

// Kleinste Winkeldifferenz in [-pi, pi].
float angleDiff(float a, float b) {
    float d = a - b;
    while (d > Kinematics::kPi) {
        d -= kTwoPi;
    }
    while (d < -Kinematics::kPi) {
        d += kTwoPi;
    }
    return d;
}

// Abstand von Punkt P zum Segment A-B.
float pointSegDist(float px, float py, float ax, float ay, float bx, float by) {
    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float len2 = abx * abx + aby * aby;
    if (len2 < 1e-12f) {
        return std::sqrt(apx * apx + apy * apy);
    }
    float t = (apx * abx + apy * aby) / len2;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const float cx = ax + t * abx;
    const float cy = ay + t * aby;
    return std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

// Orientierung (2D-Kreuzprodukt).
float orient(float ax, float ay, float bx, float by, float cx, float cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool onSegment(float ax, float ay, float bx, float by, float px, float py) {
    return std::fabs(orient(ax, ay, bx, by, px, py)) < 1e-6f &&
           px >= std::fmin(ax, bx) - 1e-6f && px <= std::fmax(ax, bx) + 1e-6f &&
           py >= std::fmin(ay, by) - 1e-6f && py <= std::fmax(ay, by) + 1e-6f;
}

// Schneiden sich die Segmente A-B und C-D?
bool segmentsIntersect(float ax, float ay, float bx, float by,
                       float cx, float cy, float dx, float dy) {
    const float o1 = orient(ax, ay, bx, by, cx, cy);
    const float o2 = orient(ax, ay, bx, by, dx, dy);
    const float o3 = orient(cx, cy, dx, dy, ax, ay);
    const float o4 = orient(cx, cy, dx, dy, bx, by);

    if (((o1 > 0.0f && o2 < 0.0f) || (o1 < 0.0f && o2 > 0.0f)) &&
        ((o3 > 0.0f && o4 < 0.0f) || (o3 < 0.0f && o4 > 0.0f))) {
        return true;
    }
    if (std::fabs(o1) < 1e-6f && onSegment(ax, ay, bx, by, cx, cy)) return true;
    if (std::fabs(o2) < 1e-6f && onSegment(ax, ay, bx, by, dx, dy)) return true;
    if (std::fabs(o3) < 1e-6f && onSegment(cx, cy, dx, dy, ax, ay)) return true;
    if (std::fabs(o4) < 1e-6f && onSegment(cx, cy, dx, dy, bx, by)) return true;
    return false;
}

}  // namespace

bool Kinematics::calculateIK(float x, float y, float& thetaL, float& thetaR) {
    return solveAndSelect(x, y, false, 0.0f, 0.0f, thetaL, thetaR);
}

bool Kinematics::calculateIK(float x, float y, float hintThetaL, float hintThetaR,
                             float& thetaL, float& thetaR) {
    return solveAndSelect(x, y, true, hintThetaL, hintThetaR, thetaL, thetaR);
}

bool Kinematics::isReachable(float x, float y) {
    float dummyL = 0.0f, dummyR = 0.0f;
    return solveAndSelect(x, y, false, 0.0f, 0.0f, dummyL, dummyR);
}

bool Kinematics::solveArmCandidates(float x, float y, float servoX, bool isLeft,
                                    float& thetaOut, float& thetaOver) {
    const float dx = x - servoX;
    const float dy = y;
    const float d = std::sqrt(dx * dx + dy * dy);

    // 1. Reichweiten-Check (Dreiecksungleichung + Sicherheitsabstand)
    const float minReach = std::fabs(L1 - L2) + EPSILON;
    const float maxReach = (L1 + L2) - EPSILON;
    if (d <= minReach || d >= maxReach) {
        return false;
    }

    // 2. Basiswinkel
    const float alpha = std::atan2(dy, dx);

    // 3. Innenwinkel (Kosinussatz); Argument defensiv auf [-1, 1] clampen
    float cosBeta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cosBeta > 1.0f) {
        cosBeta = 1.0f;
    } else if (cosBeta < -1.0f) {
        cosBeta = -1.0f;
    }
    const float beta = std::acos(cosBeta);

    // 4. Beide Kandidaten
    if (isLeft) {
        thetaOut = alpha + beta;   // Knie nach aussen
        thetaOver = alpha - beta;  // Over-Center
    } else {
        thetaOut = alpha - beta;   // Knie nach aussen
        thetaOver = alpha + beta;  // Over-Center
    }
    return true;
}

bool Kinematics::solveAndSelect(float x, float y, bool hasHint, float hintL, float hintR,
                                float& thetaL, float& thetaR) {
    float lOut, lOver, rOut, rOver;
    if (!solveArmCandidates(x, y, -D / 2.0f, true, lOut, lOver)) {
        return false;
    }
    if (!solveArmCandidates(x, y, +D / 2.0f, false, rOut, rOver)) {
        return false;
    }

    const float cL[2] = {lOut, lOver};
    const float cR[2] = {rOut, rOver};

    bool found = false;
    float bestL = 0.0f;
    float bestR = 0.0f;
    float bestDist = 0.0f;

    for (int i = 0; i < 2; ++i) {
        const float nL = normalizeAngle(cL[i], THETA_L_MIN);
        if (nL > THETA_L_MAX) {
            continue;   // Check 1: Servo-Grenze links
        }
        for (int j = 0; j < 2; ++j) {
            const float nR = normalizeAngle(cR[j], THETA_R_MIN);
            if (nR > THETA_R_MAX) {
                continue;   // Check 1: Servo-Grenze rechts
            }
            if (!collisionOk(x, y, nL, nR)) {
                continue;   // Checks 2-5
            }

            float dist = 0.0f;
            if (hasHint) {
                dist = std::fabs(angleDiff(nL, hintL)) + std::fabs(angleDiff(nR, hintR));
            }
            if (!found || dist < bestDist) {
                found = true;
                bestL = nL;
                bestR = nR;
                bestDist = dist;
            }
        }
    }

    if (!found) {
        return false;
    }
    thetaL = bestL;
    thetaR = bestR;
    return true;
}
bool Kinematics::collisionOk(float x, float y, float thetaL, float thetaR) {
    if (!servoBoxOk(x, y)) {
        return false;
    }

    const float elx = -D / 2.0f + L1 * std::cos(thetaL);
    const float ely = L1 * std::sin(thetaL);
    const float erx = +D / 2.0f + L1 * std::cos(thetaR);
    const float ery = L1 * std::sin(thetaR);

    if (!clearanceOk(x, y, elx, ely, erx, ery)) {
        return false;
    }
    if (!crossingOk(x, y, elx, ely, erx, ery)) {
        return false;
    }
    if (!hingeOrientationOk(x, y, elx, ely, erx, ery)) {
        return false;
    }
    return true;
}

bool Kinematics::servoBoxOk(float x, float y) {
    if (x >= SERVO_BOX_X_MIN && x <= SERVO_BOX_X_MAX &&
        y >= SERVO_BOX_Y_MIN && y <= SERVO_BOX_Y_MAX) {
        return false;   // P liegt in der zentralen Sperrzone
    }
    return true;
}

bool Kinematics::clearanceOk(float x, float y, float elx, float ely, float erx, float ery) {
    const float slx = -D / 2.0f;
    const float srx = +D / 2.0f;
    const float sy = 0.0f;

    // Linke Segmente (Kurbel S_L->E_L, Stange E_L->P) muessen die RECHTE Achse meiden.
    if (pointSegDist(srx, sy, slx, sy, elx, ely) < R_CLEAR) {
        return false;
    }
    if (pointSegDist(srx, sy, elx, ely, x, y) < R_CLEAR) {
        return false;
    }

    // Rechte Segmente (Kurbel S_R->E_R, Stange E_R->P) muessen die LINKE Achse meiden.
    if (pointSegDist(slx, sy, srx, sy, erx, ery) < R_CLEAR) {
        return false;
    }
    if (pointSegDist(slx, sy, erx, ery, x, y) < R_CLEAR) {
        return false;
    }
    return true;
}

bool Kinematics::crossingOk(float x, float y, float elx, float ely, float erx, float ery) {
    const float slx = -D / 2.0f;
    const float srx = +D / 2.0f;
    const float sy = 0.0f;

    // Ellbogen-Abstand (608ZZ-Gehaeuse)
    const float edx = erx - elx;
    const float edy = ery - ely;
    if (std::sqrt(edx * edx + edy * edy) < ELBOW_MIN_DIST) {
        return false;
    }

    // Kurbeln duerfen sich nicht kreuzen
    if (segmentsIntersect(slx, sy, elx, ely, srx, sy, erx, ery)) {
        return false;
    }

    // Stangen duerfen die gegenueberliegende Kurbel nicht kreuzen
    if (segmentsIntersect(elx, ely, x, y, srx, sy, erx, ery)) {
        return false;
    }
    if (segmentsIntersect(erx, ery, x, y, slx, sy, elx, ely)) {
        return false;
    }
    return true;
}

bool Kinematics::hingeOrientationOk(float x, float y, float elx, float ely, float erx, float ery) {
    // P muss auf der positiven Seite des Vektors E_L -> E_R liegen (kein Umklappen).
    const float cross = (erx - elx) * (y - ely) - (ery - ely) * (x - elx);
    return cross > 0.0f;
}

bool Kinematics::forwardKinematics(float thetaL, float thetaR, float& x, float& y) {
    // Ellbogen (Kurbelenden)
    const float elx = -D / 2.0f + L1 * std::cos(thetaL);
    const float ely = L1 * std::sin(thetaL);
    const float erx = +D / 2.0f + L1 * std::cos(thetaR);
    const float ery = L1 * std::sin(thetaR);

    const float dx = erx - elx;
    const float dy = ery - ely;
    const float d = std::sqrt(dx * dx + dy * dy);

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
