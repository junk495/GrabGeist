// ApiServer.cpp

#include <WiFi.h>        // vor WebServer.h (WebServer baut auf Network/WiFi auf)
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "ApiServer.h"

namespace {

const char* mimeFromPath(const String& path) {
    if (path.endsWith(".html"))     return "text/html";
    if (path.endsWith(".css"))      return "text/css";
    if (path.endsWith(".js"))       return "application/javascript";
    if (path.endsWith(".json"))     return "application/json";
    if (path.endsWith(".svg"))      return "image/svg+xml";
    if (path.endsWith(".png"))      return "image/png";
    if (path.endsWith(".ico"))      return "image/x-icon";
    if (path.endsWith(".manifest")) return "application/manifest+json";
    return "application/octet-stream";
}

void sendJson(WebServer& server, int code, const String& body) {
    server.send(code, "application/json", body);
}

void sendOk(WebServer& server, bool ok) {
    sendJson(server, ok ? 200 : 400, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

}  // namespace

void ApiServer::begin() {
    // Statische PWA-Assets aus LittleFS (alles außer /api/*).
    _server.onNotFound([this]() {
        const String uri = _server.uri();
        if (uri.startsWith("/api/")) {
            sendJson(_server, 404, "{\"error\":\"not found\"}");
            return;
        }
        const String path = (uri == "/") ? "/index.html" : uri;
        if (!LittleFS.exists(path)) {
            _server.send(404, "text/plain", "not found");
            return;
        }
        File f = LittleFS.open(path, "r");
        if (!f || f.isDirectory()) {
            if (f) f.close();
            _server.send(404, "text/plain", "not found");
            return;
        }
        _server.streamFile(f, mimeFromPath(path));
        f.close();
    });

    _server.on("/api/info", HTTP_GET, [this]() {
        sendJson(_server, 200, onInfo ? onInfo() : String("{}"));
    });
    _server.on("/api/status", HTTP_GET, [this]() {
        sendJson(_server, 200, onStatus ? onStatus() : String("{}"));
    });
    _server.on("/api/map", HTTP_GET, [this]() {
        sendJson(_server, 200, onMap ? onMap() : String("{}"));
    });
    _server.on("/api/reset", HTTP_GET, [this]() {
        sendJson(_server, 200, onReset ? onReset() : String("{}"));
    });

    _server.on("/api/move", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const float x = doc["x"] | 0.0f;
        const float y = doc["y"] | 0.0f;
        const bool snap = doc["snap"] | false;
        sendOk(_server, onMove ? onMove(x, y, snap) : false);
    });
    _server.on("/api/peek", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const String target = doc["target"] | "";
        sendOk(_server, onPeek ? onPeek(target) : false);
    });
    _server.on("/api/ramp", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const float rate = doc["usPerSec"] | 1000.0f;
        sendOk(_server, onRamp ? onRamp(rate) : false);
    });
    _server.on("/api/speed", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const float pct = doc["percent"] | 100.0f;
        sendOk(_server, onSpeed ? onSpeed(pct) : false);
    });
    _server.on("/api/stop", HTTP_POST, [this]() {
        sendOk(_server, onStop ? onStop() : false);
    });
    _server.on("/api/random", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const bool enabled = doc["enabled"] | true;
        sendOk(_server, onRandom ? onRandom(enabled) : false);
    });
    _server.on("/api/wifi", HTTP_POST, [this]() {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, _server.arg("plain").c_str());
        const String mode = doc["mode"] | "on";
        sendOk(_server, onWifi ? onWifi(mode) : false);
    });

    _server.on("/api/sequence", HTTP_GET, [this]() {
        sendJson(_server, 200, onSequenceGet ? onSequenceGet() : String("{}"));
    });
    _server.on("/api/sequence", HTTP_POST, [this]() {
        const bool ok = onSequenceSet ? onSequenceSet(_server.arg("plain")) : false;
        sendOk(_server, ok);
    });
    _server.on("/api/run", HTTP_POST, [this]() {
        sendOk(_server, onRun ? onRun() : false);
    });

    _server.begin();
}

void ApiServer::handle() {
    _server.handleClient();
}
