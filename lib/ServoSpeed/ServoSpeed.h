// ServoSpeed.h
// Geschwindigkeits-/Strombegrenzungs-Modell (Phase 2).
// Reine Mathematik, hardwarefrei und nativ testbar.
//
// - Strombegrenzung (intern): maxRateUsPerSec = hoechste erlaubte Puls-Slew-Rate.
// - Geschwindigkeit (pro Schritt): speedPercent 0..100, bezogen auf 100 % = Strombegrenzung.
// - Effektiv: rate = maxRateUsPerSec * speedPercent / 100.

#pragma once

struct ServoSpeed {
    // Physikalische Obergrenze des 270°-Servos: ~460°/s * 7.407 us/° ≈ 3400 us/s.
    static constexpr float SERVO_MAX_RATE_US_PER_S = 3400.0f;

    // Standard-Strombegrenzung (Brownout-Schutz).
    static constexpr float DEFAULT_LIMIT_US_PER_S = 1000.0f;

    // Geschwindigkeit auf [0, 100] klemmen.
    static constexpr float clampPercent(float percent) {
        if (percent < 0.0f) return 0.0f;
        if (percent > 100.0f) return 100.0f;
        return percent;
    }

    // Strombegrenzung auf [1, SERVO_MAX_RATE_US_PER_S] klemmen.
    static constexpr float clampRate(float rateUsPerSec) {
        if (rateUsPerSec < 1.0f) return 1.0f;
        if (rateUsPerSec > SERVO_MAX_RATE_US_PER_S) return SERVO_MAX_RATE_US_PER_S;
        return rateUsPerSec;
    }

    // Effektive Slew-Rate aus Strombegrenzung und Geschwindigkeit.
    static constexpr float rateForPercent(float maxRateUsPerSec, float percent) {
        return maxRateUsPerSec * clampPercent(percent) / 100.0f;
    }
};
