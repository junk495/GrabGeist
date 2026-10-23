// ResetLog.cpp

#include "ResetLog.h"

#include <cstring>

#include <Preferences.h>
#include <esp_system.h>

namespace {

const char* kNamespace = "grabgeist";

const char* reasonText(uint32_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

}  // namespace

void ResetLog::recordBoot() {
    Preferences prefs;
    prefs.begin(kNamespace, false);   // lesen + schreiben

    boot_ = prefs.getUInt("boot", 0) + 1;
    prefs.putUInt("boot", boot_);

    uint8_t hist[HISTORY_LEN];
    size_t len = prefs.getBytes("hist", hist, sizeof(hist));
    if (len != sizeof(hist)) {
        std::memset(hist, 0, sizeof(hist));
    }

    // Ringpuffer: aeltesten Eintrag heraus, aktuellen hinten anhaengen.
    for (size_t i = 0; i + 1 < HISTORY_LEN; ++i) {
        hist[i] = hist[i + 1];
    }
    hist[HISTORY_LEN - 1] = static_cast<uint8_t>(esp_reset_reason());
    prefs.putBytes("hist", hist, sizeof(hist));

    prefs.end();

    // Cache befuellen, damit nachfolgende Abfragen ohne NVS-Zugriff auskommen.
    loaded_ = true;
    std::memcpy(history_, hist, sizeof(hist));
}

void ResetLog::load() const {
    if (loaded_) {
        return;
    }
    Preferences prefs;
    prefs.begin(kNamespace, true);   // nur lesen
    size_t len = prefs.getBytes("hist", history_, sizeof(history_));
    if (len != sizeof(history_)) {
        std::memset(history_, 0, sizeof(history_));
    }
    boot_ = prefs.getUInt("boot", 0);
    prefs.end();
    loaded_ = true;
}

uint32_t ResetLog::lastReason() const {
    load();
    return history_[HISTORY_LEN - 1];
}

std::string ResetLog::lastReasonText() const {
    return reasonText(lastReason());
}

uint32_t ResetLog::bootCount() const {
    load();
    return boot_;
}

std::string ResetLog::format() const {
    load();
    std::string s = "RST ";
    s += reasonText(history_[HISTORY_LEN - 1]);
    s += " boot=" + std::to_string(boot_);
    s += " hist=";
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
        if (i > 0) {
            s += ",";
        }
        s += reasonText(history_[i]);
    }
    return s;
}
