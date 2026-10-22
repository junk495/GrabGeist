// BleUart.cpp

#include <Arduino.h>

#include "BleUart.h"

// Nordic UART Service (NUS)
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // Handy -> ESP
#define CHAR_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // ESP -> Handy

void BleUart::begin(const char* deviceName) {
    BLEDevice::setMTU(247);   // groessere MTU fuer den Kartentransfer
    BLEDevice::init(deviceName);

    BLEServer* server = BLEDevice::createServer();
    BLEService* service = server->createService(SERVICE_UUID);

    rxCharacteristic_ = service->createCharacteristic(
        CHAR_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rxCharacteristic_->setCallbacks(this);

    txCharacteristic_ = service->createCharacteristic(
        CHAR_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);

    service->start();

    // Service-UUID mit ins Advertising aufnehmen. Ohne diesen Aufruf wird nur der
    // Geraetename ausgestrahlt; Clients, die per Service-UUID filtern (z.B. Web
    // Bluetooth in tools/test_gui.html), finden das Geraet dann nicht.
    server->getAdvertising()->addServiceUUID(SERVICE_UUID);
    server->getAdvertising()->start();
}

void BleUart::setLineCallback(LineCallback cb) {
    lineCallback_ = cb;
}

void BleUart::sendLine(const char* line) {
    if (txCharacteristic_ == nullptr || line == nullptr) {
        return;
    }
    String payload = String(line) + "\n";
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
            delay(10);   // Pacen gegen Web-Bluetooth-Notification-Verlust
        }
    }
}

void BleUart::onWrite(BLECharacteristic* characteristic) {
    if (characteristic != rxCharacteristic_) {
        return;
    }
    String value = characteristic->getValue();
    buffer_ += value.c_str();

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
