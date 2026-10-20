// BleUart.h
// BLE-UART (Nordic UART Service) fuer die einfache Test-Steuerung.
// ESP32-spezifisch (nutzt das BLE-API des Arduino-Cores) — nicht nativ testbar.

#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <string>

class BleUart : public BLECharacteristicCallbacks {
public:
    // Rueckruf: wird aufgerufen, wenn eine komplette Zeile (ohne \n) empfangen wurde.
    using LineCallback = void (*)(const char* line);

    void begin(const char* deviceName);
    void setLineCallback(LineCallback cb);

    // Zeile an das Handy senden (haengt \n an)
    void sendLine(const char* line);

    // BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* characteristic) override;

private:
    BLECharacteristic* rxCharacteristic_ = nullptr;   // Handy -> ESP
    BLECharacteristic* txCharacteristic_ = nullptr;   // ESP -> Handy (notify)
    LineCallback lineCallback_ = nullptr;
    std::string buffer_;
};
