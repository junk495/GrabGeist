// ApiServer.h
// Minimaler HTTP/JSON-Server (synchron, laeuft in loop()).
// Liefert die PWA aus LittleFS und die /api/*-Endpunkte.

#pragma once

#include <WebServer.h>
#include <functional>
#include <string>

class ApiServer {
public:
    std::function<String()> onInfo;
    std::function<String()> onStatus;
    std::function<String()> onMap;
    std::function<String()> onReset;
    std::function<bool(float x, float y, bool snap)> onMove;
    std::function<bool(const String& target)> onPeek;
    std::function<bool(float rate)> onRamp;
    std::function<bool(float percent)> onSpeed;
    std::function<bool()> onStop;
    std::function<bool(bool enabled)> onRandom;
    std::function<bool(const String& mode)> onWifi;
    std::function<String()> onSequenceGet;
    std::function<bool(const String& json)> onSequenceSet;
    std::function<bool()> onRun;

    void begin();
    void handle();

private:
    WebServer _server;
};
