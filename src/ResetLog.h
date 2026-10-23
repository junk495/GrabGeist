// ResetLog.h
// Persistente Reset-Diagnose: zeichnet den Reset-Grund jedes Boots in NVS auf,
// damit er spaeter ohne Seriellen Monitor (z.B. per BLE) abgefragt werden kann.
// ESP32-spezifisch (NVS/Preferences) - nicht nativ testbar.

#pragma once

#include <cstdint>
#include <string>

class ResetLog {
public:
    static constexpr size_t HISTORY_LEN = 8;

    // Beim Boot so frueh wie moeglich aufrufen: haengt den aktuellen Reset-Grund
    // (esp_reset_reason) an den NVS-Ringpuffer an und erhoeht den Bootzaehler.
    void recordBoot();

    // Fertige Diagnose-Zeile, z.B.:
    //   "RST BROWNOUT boot=3 hist=POWERON,BROWNOUT,BROWNOUT"
    std::string format() const;

    // Lesbarer Text des zuletzt aufgetretenen Reset-Grunds.
    std::string lastReasonText() const;

    // Rohwerte fuer eigene Auswertung.
    uint32_t lastReason() const;
    uint32_t bootCount() const;

private:
    void load() const;

    mutable bool loaded_ = false;
    mutable uint8_t history_[HISTORY_LEN] = {0};
    mutable uint32_t boot_ = 0;
};
