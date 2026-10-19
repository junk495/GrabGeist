// ServoController.h
// Bildet Kinematik-Winkel (Kurbelwinkel theta) auf PWM-Pulsbreiten ab.
// Reine Umrechnung - hardwareunabhaengig. Der eigentliche PWM-Write erfolgt im Sketch.

#pragma once

#include <cstdint>

class ServoController {
public:
    // 270-Grad-Digital-Servo: 500 us = 0 Grad, 1500 us = Mitte (135 Grad), 2500 us = 270 Grad
    static constexpr float MIN_PULSE_US    = 500.0f;
    static constexpr float MAX_PULSE_US    = 2500.0f;
    static constexpr float CENTER_PULSE_US = 1500.0f;

    static constexpr float kPi        = 3.14159265358979323846f;
    static constexpr float TRAVEL_DEG = 270.0f;                          // Stellbereich
    static constexpr float TRAVEL_RAD = TRAVEL_DEG * kPi / 180.0f;       // in Radiant
    static constexpr float US_PER_RAD = (MAX_PULSE_US - MIN_PULSE_US) / TRAVEL_RAD;

    // Kalibrierung je Servo setzen.
    //   zeroOffsetRad: Kinematik-Winkel [rad], bei dem der Servo auf Mitte (1500 us) steht.
    //   invert:        true, wenn der Servo gespiegelt montiert ist (Drehrichtung umkehren).
    void setCalibrationLeft(float zeroOffsetRad, bool invert);
    void setCalibrationRight(float zeroOffsetRad, bool invert);

    // Kernfunktion: Kurbelwinkel -> Pulsbreite in Mikrosekunden (geklemmt auf [500, 2500]).
    uint16_t angleToPulseUsLeft(float thetaRad) const;
    uint16_t angleToPulseUsRight(float thetaRad) const;

private:
    struct Calibration {
        float zeroOffsetRad = 0.0f;
        bool  invert        = false;
    };

    uint16_t angleToPulseUs(float thetaRad, const Calibration& cal) const;

    Calibration left_;
    Calibration right_;
};
