// ServoController.cpp

#include "ServoController.h"

#include <cmath>

void ServoController::setCalibrationLeft(float zeroOffsetRad, bool invert) {
    left_.zeroOffsetRad = zeroOffsetRad;
    left_.invert        = invert;
}

void ServoController::setCalibrationRight(float zeroOffsetRad, bool invert) {
    right_.zeroOffsetRad = zeroOffsetRad;
    right_.invert        = invert;
}

uint16_t ServoController::angleToPulseUsLeft(float thetaRad) const {
    return angleToPulseUs(thetaRad, left_);
}

uint16_t ServoController::angleToPulseUsRight(float thetaRad) const {
    return angleToPulseUs(thetaRad, right_);
}

uint16_t ServoController::angleToPulseUs(float thetaRad, const Calibration& cal) const {
    // Relativer Winkel zur kalibrierten Null-Lage
    float delta = thetaRad - cal.zeroOffsetRad;

    // Drehrichtung umkehren, falls der Servo gespiegelt montiert ist
    if (cal.invert) {
        delta = -delta;
    }

    // Winkel -> Pulsbreite
    float pulse = CENTER_PULSE_US + delta * US_PER_RAD;

    // Auf den gueltigen Bereich klemmen
    if (pulse < MIN_PULSE_US) {
        pulse = MIN_PULSE_US;
    } else if (pulse > MAX_PULSE_US) {
        pulse = MAX_PULSE_US;
    }

    return static_cast<uint16_t>(std::round(pulse));
}

float ServoController::pulseToAngleUsLeft(float pulseUs) const {
    return pulseToAngleUs(pulseUs, left_);
}

float ServoController::pulseToAngleUsRight(float pulseUs) const {
    return pulseToAngleUs(pulseUs, right_);
}

float ServoController::pulseToAngleUs(float pulseUs, const Calibration& cal) const {
    float delta = (pulseUs - CENTER_PULSE_US) / US_PER_RAD;
    if (cal.invert) {
        delta = -delta;
    }
    return cal.zeroOffsetRad + delta;
}
