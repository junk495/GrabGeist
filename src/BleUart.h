// BleUart.h
// BLE-UART (Nordic UART Service). Nutzt NimBLE — stabiler mit Android als Bluedroid.
// ESP32-spezifisch — nicht nativ testbar.

#pragma once

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEConnInfo.h>

#include <string>

class BleUart : public NimBLECharacteristicCallbacks {
public:
    // Rueckruf: wird aufgerufen, wenn eine komplette Zeile (ohne \n) empfangen wurde.
    using LineCallback = void (*)(const char* line);

    void begin(const char* deviceName);
    void setLineCallback(LineCallback cb);

    // Zeile an das Handy senden (haengt \n an)
    void sendLine(const char* line);

    // Großen Textblock in MTU-gerechten Chunks senden (fuer die Arbeitsraum-Karte).
    void sendChunked(const char* data, size_t len);

    // NimBLECharacteristicCallbacks
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override;

private:
    NimBLECharacteristic* rxCharacteristic_ = nullptr;   // Handy -> ESP
    NimBLECharacteristic* txCharacteristic_ = nullptr;   // ESP -> Handy (notify)
    LineCallback lineCallback_ = nullptr;
    std::string buffer_;
};
