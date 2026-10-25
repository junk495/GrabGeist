// BleUart.cpp

#include <Arduino.h>

#include "BleUart.h"

// Nordic UART Service (NUS)
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // Handy -> ESP
#define CHAR_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // ESP -> Handy

namespace {
// Startet das Advertising nach einem Verbindungsabbruch neu. Ohne das bleibt
// der ESP32 nach einem Disconnect "unsichtbar": Das Handy findet ihn erst nach
// einem Reset wieder (auch der Auto-Reconnect der PWA schlaegt dann fehl).
class ServerCallbacks : public NimBLEServerCallbacks {
public:
    NimBLEAdvertising* advertising = nullptr;

    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        // Advertising stoppt beim Verbinden automatisch.
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        if (advertising != nullptr) {
            advertising->start();
        }
    }
};
}  // namespace

void BleUart::begin(const char* deviceName) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setMTU(247);   // groessere MTU fuer den Kartentransfer

    NimBLEServer* server = NimBLEDevice::createServer();

    // Advertising nach Disconnect automatisch wieder starten.
    auto* serverCallbacks = new ServerCallbacks();
    server->setCallbacks(serverCallbacks);

    NimBLEService* service = server->createService(SERVICE_UUID);

    rxCharacteristic_ = service->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxCharacteristic_->setCallbacks(this);

    txCharacteristic_ = service->createCharacteristic(
        CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY);

    // Service-UUID mit ins Advertising aufnehmen. Ohne diesen Aufruf wird nur der
    // Geraetename ausgestrahlt; Clients, die per Service-UUID filtern (z.B. Web
    // Bluetooth in tools/test_gui.html), finden das Geraet dann nicht.
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setName(deviceName);   // Name ins Advertising (fuer die Geraeteliste)
    advertising->start();
    serverCallbacks->advertising = advertising;
}

void BleUart::setLineCallback(LineCallback cb) {
    lineCallback_ = cb;
}

void BleUart::sendLine(const char* line) {
    if (txCharacteristic_ == nullptr || line == nullptr) {
        return;
    }
    std::string payload = std::string(line) + "\n";
    txCharacteristic_->setValue(payload);
    txCharacteristic_->notify();

    // Kurze Pause zwischen Notifications. Ohne Pause verliert/verschmilzt Web
    // Bluetooth (Browser) bei schnellem Versand vieler Zeilen (z.B. der
    // Arbeitsraum-Karte) einzelne Notifications -> Karte bleibt unvollstaendig.
    delay(15);
}

void BleUart::sendChunked(const char* data, size_t len) {
    if (txCharacteristic_ == nullptr || data == nullptr) {
        return;
    }
    // 180 Byte pro Notification: sicher unter der MTU (247 -> max. 244), aber
    // auch noch unter 185-3=182, falls der Browser eine kleinere MTU aushandelt.
    const size_t CHUNK = 180;
    size_t off = 0;
    while (off < len) {
        const size_t n = (len - off) < CHUNK ? (len - off) : CHUNK;
        txCharacteristic_->setValue((const uint8_t*)(data + off), n);
        txCharacteristic_->notify();
        off += n;
        if (off < len) {
            delay(20);   // Pacen gegen Web-Bluetooth-Notification-Verlust
        }
    }
}

void BleUart::onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) {
    if (characteristic != rxCharacteristic_) {
        return;
    }
    std::string value = characteristic->getValue();
    buffer_ += value;

    // Komplette Zeilen (durch \n getrennt) herausziehen und ausliefern
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string line = buffer_.substr(0, pos);
        buffer_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (lineCallback_ != nullptr) {
            lineCallback_(line.c_str());
        }
    }
}
