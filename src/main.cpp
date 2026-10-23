// main.cpp — GrabGeist Firmware (ESP32-S3)
//
// Test-Steuerung ueber BLE (Handy):
//   BLE -> CommandParser -> MotionPlanner -> Kinematics -> ServoController -> LEDC

#include <Arduino.h>
#include <esp_random.h>   // esp_random() fuer den Zufallsmodus

#include <CommandParser.h>
#include <Kinematics.h>
#include <MotionPlanner.h>
#include <ServoController.h>
#include <Workspace.h>

#include <cstring>
#include <string>
#include <vector>

#include "BleUart.h"
#include "ResetLog.h"

// ---------------------------------------------------------------------------
// Pinbelegung (am Modell verifiziert).
// GPIOs ohne Konflikt mit Boot-Strapping-/Flash-/PSRAM-/UART0-/USB-Belegung.
// ---------------------------------------------------------------------------
static const uint8_t PIN_SERVO_LEFT  = 4;
static const uint8_t PIN_SERVO_RIGHT = 5;

// ---------------------------------------------------------------------------
// PWM (LEDC) fuer 270-Grad-RC-Servos: 50 Hz, 14 Bit.
// Hinweis: ESP32-S3-LEDC erlaubt maximal 14 Bit Aufloesung.
// Pulsbreite 500..2500 us wird auf Duty 0..2^14 abgebildet.
// ---------------------------------------------------------------------------
static const uint32_t SERVO_FREQ_HZ   = 50;
static const uint8_t  SERVO_RES_BITS  = 14;
static const uint32_t SERVO_PERIOD_US = 1000000u / SERVO_FREQ_HZ;   // 20000 us
static const uint32_t SERVO_DUTY_FS   = (1u << SERVO_RES_BITS);     // 2^14 = 16384

static const uint32_t LOOP_INTERVAL_MS = 20;   // 50 Hz

// ---------------------------------------------------------------------------
// Servo-Anfahrrampe (Rate-Limit): maximale Pulsbreiten-Aenderung pro Sekunde [us/s].
// 500..2500 us entsprechen 270° -> 750 us/s ~ 100°/s Kurbelgeschwindigkeit.
// Je kleiner, desto sanfter der Anlauf (weniger Stromspitzen beim 5V-/LiPo-Betrieb).
// 0 = keine Begrenzung (Servos springen sofort). Zur Laufzeit per Befehl "RAMP <us/s>"
// aenderbar ("RAMP" ohne Wert fragt den aktuellen Wert ab).
// ---------------------------------------------------------------------------
static constexpr float SERVO_RAMP_US_PER_S_DEFAULT = 750.0f;
static float servoRampUsPerS = SERVO_RAMP_US_PER_S_DEFAULT;

// ---------------------------------------------------------------------------
// Preset-Koordinaten (am Modell verifiziert).
// ---------------------------------------------------------------------------
static const float POS_LEFT_X  = -50.0f, POS_LEFT_Y  = 150.0f;
static const float POS_RIGHT_X = 50.0f,  POS_RIGHT_Y = 150.0f;

// "Versteckt" als direkte Servo-Position (am Modell verifiziert).
// Kollisionsgrenze: tiefer (= Kurbeln weiter aussen) beruehren sich die Arme.
static const uint16_t HIDE_PULSE_L = 1580;
static const uint16_t HIDE_PULSE_R = 1410;

// Software-Bausteine
static MotionPlanner   planner;
static ServoController servos;
static BleUart         ble;
static Workspace       workspace;
static ResetLog        resetLog;

// Zuletzt an die Servo-Pins ausgegebene PWM-Pulsbreiten (fuer die Monitor-Ausgabe).
static uint16_t lastPulseL = 0;
static uint16_t lastPulseR = 0;
static bool     lastIkValid = false;
static bool     directMode = false;   // true = Servos direkt ansteuern (RAW), IK/Planner pausiert

// Servo-Soll (Ziel) und tatsaechlich ausgegebene Pulsbreite (durch die Rampe begrenzt).
static uint16_t desiredPulseL = 0, desiredPulseR = 0;
static uint16_t outputPulseL = 0, outputPulseR = 0;

// ---------------------------------------------------------------------------
// Auto-Ablauf: links -> oben -> rechts -> versteckt -> links -> ...
// 1 s Uebergang, 2 s Halten pro Position.
// ---------------------------------------------------------------------------
static const uint32_t AUTO_MOVE_MS = 1000;
static const uint32_t AUTO_HOLD_MS = 2000;
static const int      AUTO_STEP_COUNT = 4;

static bool     autoActive = false;
static int      autoStep = 0;
static bool     autoMoving = false;      // true = Uebergang, false = Halten
static uint32_t autoPhaseStart = 0;
static uint16_t autoStartL = 0, autoStartR = 0;
static uint16_t autoTargetL = 0, autoTargetR = 0;

// ---------------------------------------------------------------------------
// Zufallsmodus: zufaellig hervor- und zurueckschauen (Versteckt / Links / Rechts / Oben).
// ---------------------------------------------------------------------------
static const uint32_t RANDOM_PEEK_HOLD_MIN_MS = 800;
static const uint32_t RANDOM_PEEK_HOLD_MAX_MS = 2500;
static const uint32_t RANDOM_HIDE_HOLD_MIN_MS = 1500;
static const uint32_t RANDOM_HIDE_HOLD_MAX_MS = 5000;

enum class RandomStage { PEEK_MOVE, PEEK_HOLD, HIDE_MOVE, HIDE_HOLD };

static bool        randomActive = false;
static RandomStage randomStage = RandomStage::HIDE_HOLD;
static uint32_t    randomStageStart = 0;
static uint32_t    randomHoldMs = 0;

// ---------------------------------------------------------------------------
// Record/Playback: Servo-Zielpulsbreiten aufnehmen und als Schleife abspielen.
// ---------------------------------------------------------------------------
static const uint32_t REC_INTERVAL_MS = 50;      // 20 Hz Abtastrate
static const size_t   REC_MAX_SAMPLES = 1200;    // 60 s @ 20 Hz

static uint16_t recPulseL[REC_MAX_SAMPLES];
static uint16_t recPulseR[REC_MAX_SAMPLES];
static size_t   recCount = 0;
static bool     recActive = false;
static uint32_t lastRecMs = 0;

static bool     playbackActive = false;
static size_t   playbackIndex = 0;
static uint32_t lastPlaybackMs = 0;

// ---------------------------------------------------------------------------
// Script: Text-Ablauf (eine Zeile = ein Befehl), vom PC hochgeladen, als Schleife abspielbar.
// ---------------------------------------------------------------------------
static const size_t SCRIPT_MAX_LINES = 200;

static std::vector<std::string> scriptLines;
static bool   scriptLoading = false;
static bool   scriptActive = false;
static size_t scriptIndex = 0;

enum class ScriptWait { NONE, DELAY, ARRIVAL };
static ScriptWait scriptWait = ScriptWait::NONE;
static uint32_t   scriptWaitStart = 0;
static uint32_t   scriptWaitMs = 0;

// ---------------------------------------------------------------------------
void servoWriteUs(uint8_t pin, uint16_t pulseUs) {
    const uint32_t duty = (uint32_t)pulseUs * SERVO_DUTY_FS / SERVO_PERIOD_US;
    if (!ledcWrite(pin, duty)) {
        Serial.printf("ledcWrite fehlgeschlagen (pin=%u duty=%u)\n", pin, (unsigned)duty);
    }
}

// ---------------------------------------------------------------------------
// Ziel-Pulsbreiten setzen (Soll). Die tatsaechliche Ausgabe naehert sich diesem
// Ziel in updateServoRamp() mit begrenzter Geschwindigkeit an.
// ---------------------------------------------------------------------------
void setTargetPulses(uint16_t pulseL, uint16_t pulseR) {
    desiredPulseL = pulseL;
    desiredPulseR = pulseR;
    lastPulseL = pulseL;
    lastPulseR = pulseR;
    lastIkValid = true;
}

// Sofort auf eine Position setzen (ohne Rampe) - fuer Start/Homing.
void setServoInstant(uint16_t pulseL, uint16_t pulseR) {
    directMode = true;
    setTargetPulses(pulseL, pulseR);
    outputPulseL = pulseL;
    outputPulseR = pulseR;
    servoWriteUs(PIN_SERVO_LEFT,  pulseL);
    servoWriteUs(PIN_SERVO_RIGHT, pulseR);
}

// ---------------------------------------------------------------------------
// Servo-Ausgabe mit Rate-Limit: bewegt die tatsaechlich ausgegebenen Pulsbreiten
// pro Zeitschritt maximal um servoRampUsPerS * dt in Richtung Ziel. Das begrenzt
// die Servo-Geschwindigkeit/Beschleunigung und damit die Stromspitzen.
// ---------------------------------------------------------------------------
void updateServoRamp(float dt) {
    if (servoRampUsPerS <= 0.0f) {
        // Keine Begrenzung -> direkt ans Ziel springen.
        outputPulseL = desiredPulseL;
        outputPulseR = desiredPulseR;
    } else {
        const float maxStep = servoRampUsPerS * dt;
        const float dl = (float)desiredPulseL - (float)outputPulseL;
        const float dr = (float)desiredPulseR - (float)outputPulseR;
        if (dl > maxStep)       outputPulseL = (uint16_t)((float)outputPulseL + maxStep);
        else if (dl < -maxStep) outputPulseL = (uint16_t)((float)outputPulseL - maxStep);
        else                    outputPulseL = desiredPulseL;
        if (dr > maxStep)       outputPulseR = (uint16_t)((float)outputPulseR + maxStep);
        else if (dr < -maxStep) outputPulseR = (uint16_t)((float)outputPulseR - maxStep);
        else                    outputPulseR = desiredPulseR;
    }

    servoWriteUs(PIN_SERVO_LEFT,  outputPulseL);
    servoWriteUs(PIN_SERVO_RIGHT, outputPulseR);
}

// Karte als einen grossen Textblock senden: statt 75 einzelner Zeilen wird der
// Karten-Versand als nicht-blockierende Zustandsmaschine: statt im BLE-Callback
// mit delay() zu blockieren (blockiert den BLE-Host-Task -> Sende-Credits gehen
// aus -> Karte bleibt halb), wird hier nur der Text aufgebaut und dann im Loop
// Chunk fuer Chunk gesendet.
static volatile bool mapSending = false;
static std::string mapData;
static size_t      mapOffset = 0;
static uint32_t    lastMapChunkMs = 0;
static const size_t   MAP_CHUNK = 180;
static const uint32_t MAP_CHUNK_MS = 20;

void startMapSend() {
    std::string out;
    out.reserve(workspace.width() * (workspace.height() + 1) + 64);

    char hdr[64];
    snprintf(hdr, sizeof(hdr), "MAP %d %d %d %d %d\n",
             workspace.width(), workspace.height(),
             (int)workspace.xMin(), (int)workspace.yMin(), (int)workspace.step());
    out += hdr;

    const std::vector<uint8_t>& grid = workspace.grid();
    for (int cy = 0; cy < workspace.height(); ++cy) {
        for (int cx = 0; cx < workspace.width(); ++cx) {
            out += (grid[cy * workspace.width() + cx] ? '1' : '0');
        }
        out += '\n';
    }

    mapData = std::move(out);
    mapOffset = 0;
    mapSending = true;
    lastMapChunkMs = millis();
}

// Einen Chunk senden (vom Loop aufgerufen; sendChunked mit <=180 Byte sendet
// genau eine Notification ohne delay()).
void updateMapSend() {
    if (!mapSending) {
        return;
    }
    if (millis() - lastMapChunkMs < MAP_CHUNK_MS) {
        return;
    }

    const size_t remaining = mapData.size() - mapOffset;
    const size_t n = remaining < MAP_CHUNK ? remaining : MAP_CHUNK;
    ble.sendChunked(mapData.data() + mapOffset, n);
    mapOffset += n;
    lastMapChunkMs = millis();

    if (mapOffset >= mapData.size()) {
        mapSending = false;
    }
}

// Aktuelle Servo-Pulsbreiten an das Handy melden (nach abgeschlossenem Wechsel).
void sendPositionUpdate() {
    if (mapSending) {
        return;   // nicht in den Kartentransfer interleaven
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "PWM %u %u",
             (unsigned)lastPulseL, (unsigned)lastPulseR);
    ble.sendLine(buf);
}

// Zuletzt gueltige Kurbelwinkel (Hysterese-Hint fuer die bimodale IK).
static float lastThetaL = 0.0f;
static float lastThetaR = 0.0f;

void moveToPosition(float x, float y) {
    float thetaL = 0.0f, thetaR = 0.0f;
    // Hysterese: die naechste gueltige Konfiguration zu den aktuellen Winkeln waehlen.
    if (Kinematics::calculateIK(x, y, lastThetaL, lastThetaR, thetaL, thetaR)) {
        lastThetaL = thetaL;
        lastThetaR = thetaR;
        setTargetPulses(servos.angleToPulseUsLeft(thetaL),
                        servos.angleToPulseUsRight(thetaR));
    } else {
        lastIkValid = false;   // ungueltige Position -> Servos halten letzte Stellung
    }
}

// ---------------------------------------------------------------------------
// Servos direkt auf feste Pulsbreiten setzen (umgeht IK/Planner).
// ---------------------------------------------------------------------------
void setServoDirect(uint16_t pulseL, uint16_t pulseR) {
    directMode = true;
    setTargetPulses(pulseL, pulseR);
}

// ---------------------------------------------------------------------------
// Auto-Modus (nicht-blockierend, wird in loop() aufgerufen).
// ---------------------------------------------------------------------------

// IK-Pulsbreiten fuer eine Position berechnen. false, wenn unerreichbar.
bool ikPulses(float x, float y, uint16_t& pulseL, uint16_t& pulseR) {
    float thetaL = 0.0f, thetaR = 0.0f;
    if (!Kinematics::calculateIK(x, y, thetaL, thetaR)) {
        return false;
    }
    pulseL = servos.angleToPulseUsLeft(thetaL);
    pulseR = servos.angleToPulseUsRight(thetaR);
    return true;
}

// Ziel-Pulsbreiten fuer einen Auto-Schritt bestimmen.
bool autoTargetPulses(int step, uint16_t& pulseL, uint16_t& pulseR) {
    switch (step) {
        case 0:  // links
            return ikPulses(POS_LEFT_X, POS_LEFT_Y, pulseL, pulseR);
        case 1:  // oben (beide Kurbeln senkrecht)
            pulseL = servos.angleToPulseUsLeft(Kinematics::degToRad(90.0f));
            pulseR = servos.angleToPulseUsRight(Kinematics::degToRad(90.0f));
            return true;
        case 2:  // rechts
            return ikPulses(POS_RIGHT_X, POS_RIGHT_Y, pulseL, pulseR);
        case 3:  // versteckt
            pulseL = HIDE_PULSE_L;
            pulseR = HIDE_PULSE_R;
            return true;
        default:
            return false;
    }
}

void startAutoMode() {
    autoActive = true;
    randomActive = false;
    playbackActive = false;
    scriptActive = false;
    autoStep = 0;
    if (!autoTargetPulses(autoStep, autoTargetL, autoTargetR)) {
        autoTargetL = HIDE_PULSE_L;
        autoTargetR = HIDE_PULSE_R;
    }
    autoStartL = lastPulseL;
    autoStartR = lastPulseR;
    autoMoving = true;
    autoPhaseStart = millis();
}

void updateAutoMode() {
    const uint32_t now = millis();
    const uint32_t elapsed = now - autoPhaseStart;

    if (autoMoving) {
        // Linearer Uebergang start -> target (1 s)
        float t = (float)elapsed / (float)AUTO_MOVE_MS;
        if (t > 1.0f) t = 1.0f;
        uint16_t l = (uint16_t)((float)autoStartL + ((float)autoTargetL - (float)autoStartL) * t);
        uint16_t r = (uint16_t)((float)autoStartR + ((float)autoTargetR - (float)autoStartR) * t);
        setTargetPulses(l, r);

        if (elapsed >= AUTO_MOVE_MS) {
            autoMoving = false;
            autoPhaseStart = now;
        }
    } else {
        // Halten (2 s)
        if (elapsed >= AUTO_HOLD_MS) {
            autoStep = (autoStep + 1) % AUTO_STEP_COUNT;
            if (!autoTargetPulses(autoStep, autoTargetL, autoTargetR)) {
                autoTargetL = HIDE_PULSE_L;
                autoTargetR = HIDE_PULSE_R;
            }
            autoStartL = lastPulseL;
            autoStartR = lastPulseR;
            autoMoving = true;
            autoPhaseStart = now;
        }
    }
}

// ---------------------------------------------------------------------------
// Zufallsmodus (nicht-blockierend, wird in loop() aufgerufen).
// ---------------------------------------------------------------------------
static uint32_t randBetween(uint32_t minIncl, uint32_t maxExcl) {
    if (maxExcl <= minIncl) {
        return minIncl;
    }
    return minIncl + (esp_random() % (maxExcl - minIncl));
}

// Zufaellige Peek-Position waehlen (links / rechts / oben).
static void randomPeekPulses(uint16_t& pulseL, uint16_t& pulseR) {
    const uint32_t pick = randBetween(0, 3);
    if (pick == 0 && ikPulses(POS_LEFT_X, POS_LEFT_Y, pulseL, pulseR)) {
        return;
    }
    if (pick == 1 && ikPulses(POS_RIGHT_X, POS_RIGHT_Y, pulseL, pulseR)) {
        return;
    }
    // Oben (beide Kurbeln senkrecht) als Standard/Fallback.
    pulseL = servos.angleToPulseUsLeft(Kinematics::degToRad(90.0f));
    pulseR = servos.angleToPulseUsRight(Kinematics::degToRad(90.0f));
}

void startRandomMode() {
    randomActive = true;
    autoActive = false;
    playbackActive = false;
    scriptActive = false;
    directMode = false;
    randomStage = RandomStage::HIDE_MOVE;   // erst verstecken, dann loslegen
    randomStageStart = millis();
    setTargetPulses(HIDE_PULSE_L, HIDE_PULSE_R);
}

void updateRandomMode() {
    const uint32_t now = millis();
    switch (randomStage) {
        case RandomStage::PEEK_MOVE:
            // Warten, bis die Rampe das Ziel erreicht hat.
            if (outputPulseL == desiredPulseL && outputPulseR == desiredPulseR) {
                randomHoldMs = randBetween(RANDOM_PEEK_HOLD_MIN_MS, RANDOM_PEEK_HOLD_MAX_MS);
                randomStage = RandomStage::PEEK_HOLD;
                randomStageStart = now;
            }
            break;

        case RandomStage::PEEK_HOLD:
            if (now - randomStageStart >= randomHoldMs) {
                setTargetPulses(HIDE_PULSE_L, HIDE_PULSE_R);
                randomStage = RandomStage::HIDE_MOVE;
            }
            break;

        case RandomStage::HIDE_MOVE:
            if (outputPulseL == desiredPulseL && outputPulseR == desiredPulseR) {
                randomHoldMs = randBetween(RANDOM_HIDE_HOLD_MIN_MS, RANDOM_HIDE_HOLD_MAX_MS);
                randomStage = RandomStage::HIDE_HOLD;
                randomStageStart = now;
            }
            break;

        case RandomStage::HIDE_HOLD:
            if (now - randomStageStart >= randomHoldMs) {
                uint16_t pl, pr;
                randomPeekPulses(pl, pr);
                setTargetPulses(pl, pr);
                randomStage = RandomStage::PEEK_MOVE;
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// Record/Playback (nicht-blockierend).
// ---------------------------------------------------------------------------
void startRecording() {
    recCount = 0;
    recActive = true;
    lastRecMs = 0;
    playbackActive = false;   // nicht waehrend der Wiedergabe aufnehmen
}

void updateRecording() {
    if (!recActive) {
        return;
    }
    const uint32_t now = millis();
    if (now - lastRecMs < REC_INTERVAL_MS) {
        return;
    }
    lastRecMs = now;

    if (recCount >= REC_MAX_SAMPLES) {
        recActive = false;
        if (!mapSending) {
            char buf[48];
            snprintf(buf, sizeof(buf), "REC full %u", (unsigned)recCount);
            ble.sendLine(buf);
        }
        return;
    }

    recPulseL[recCount] = desiredPulseL;
    recPulseR[recCount] = desiredPulseR;
    ++recCount;
}

void startPlayback() {
    if (recCount == 0) {
        return;
    }
    playbackActive = true;
    playbackIndex = 0;
    lastPlaybackMs = 0;
    recActive = false;
    autoActive = false;
    randomActive = false;
    scriptActive = false;
    directMode = false;
}

void updatePlayback() {
    const uint32_t now = millis();
    if (now - lastPlaybackMs < REC_INTERVAL_MS) {
        return;
    }
    lastPlaybackMs = now;
    setTargetPulses(recPulseL[playbackIndex], recPulseR[playbackIndex]);
    playbackIndex = (playbackIndex + 1) % recCount;   // als Schleife wiederholen
}

// ---------------------------------------------------------------------------
// BLE-Empfang: eine Zeile -> Befehl parsen und ausfuehren.
// (Wird aus dem BLE-Kontext aufgerufen; fuer den Testbetrieb reicht das.)
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Workspace-Karte als Text an das Handy senden (Header + '0'/'1'-Zeilen).
// ---------------------------------------------------------------------------
void sendMap() {
    char hdr[64];
    snprintf(hdr, sizeof(hdr), "MAP %d %d %.0f %.0f %.0f",
             workspace.width(), workspace.height(),
             workspace.xMin(), workspace.yMin(), workspace.step());
    ble.sendLine(hdr);

    const auto& grid = workspace.grid();
    std::string row;
    row.reserve(workspace.width());
    for (int cy = 0; cy < workspace.height(); ++cy) {
        row.clear();
        for (int cx = 0; cx < workspace.width(); ++cx) {
            row += (grid[cy * workspace.width() + cx] != 0) ? '1' : '0';
        }
        ble.sendLine(row.c_str());
    }
    ble.sendLine("MAP_DONE");
}

// ---------------------------------------------------------------------------
// Befehle ausfuehren (ohne BLE-Antwort) - von handleLine UND vom Script benutzt.
// ---------------------------------------------------------------------------
void doMoveCommand(float x, float y, bool snap) {
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    directMode = false;

    // Ziel auf den naechsten gueltigen Punkt snappen
    float tx = x, ty = y;
    workspace.nearestValid(x, y, tx, ty);

    if (snap) {
        planner.moveTo(tx, ty, MotionPlanner::MoveType::SNAP);
    } else {
        // Pfad vom aktuellen Punkt (ggf. gesnappt) zum Ziel (A* um die Sperrzone)
        float sx = planner.x(), sy = planner.y();
        workspace.nearestValid(sx, sy, sx, sy);
        std::vector<Point> path;
        if (workspace.findPath(sx, sy, tx, ty, path)) {
            planner.moveAlongPath(path, MotionPlanner::MoveType::SMOOTH);
        } else {
            planner.moveTo(tx, ty, MotionPlanner::MoveType::SMOOTH);
        }
    }
}

void doPeekCommand(Command::Peek peek) {
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    directMode = false;
    switch (peek) {
        case Command::Peek::LEFT:
            planner.moveTo(POS_LEFT_X, POS_LEFT_Y, MotionPlanner::MoveType::SMOOTH);
            break;
        case Command::Peek::RIGHT:
            planner.moveTo(POS_RIGHT_X, POS_RIGHT_Y, MotionPlanner::MoveType::SMOOTH);
            break;
        case Command::Peek::TOP:
            // Referenz: beide Kurbeln senkrecht nach oben (theta = 90°).
            setServoDirect(servos.angleToPulseUsLeft(Kinematics::degToRad(90.0f)),
                           servos.angleToPulseUsRight(Kinematics::degToRad(90.0f)));
            break;
        case Command::Peek::HIDE:
            setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
            break;
    }
}

void doRawCommand(uint16_t pulseL, uint16_t pulseR) {
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    setServoDirect(pulseL, pulseR);
}

// ---------------------------------------------------------------------------
// Script (nicht-blockierend): Zeile fuer Zeile ausfuehren, als Schleife.
// ---------------------------------------------------------------------------
void startScript() {
    scriptActive = true;
    scriptIndex = 0;
    scriptWait = ScriptWait::NONE;
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    recActive = false;
    directMode = false;
}

void updateScript() {
    if (!scriptActive) {
        return;
    }
    const uint32_t now = millis();

    if (scriptWait == ScriptWait::DELAY) {
        if (now - scriptWaitStart < scriptWaitMs) {
            return;
        }
        scriptWait = ScriptWait::NONE;
    } else if (scriptWait == ScriptWait::ARRIVAL) {
        if (outputPulseL != desiredPulseL || outputPulseR != desiredPulseR) {
            return;
        }
        scriptWait = ScriptWait::NONE;
    }

    while (scriptIndex < scriptLines.size()) {
        const std::string& line = scriptLines[scriptIndex++];

        // Kommentare (#) und Leerzeilen ueberspringen.
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') {
            ++s;
        }
        if (*s == '\0' || *s == '#') {
            continue;
        }

        Command cmd;
        if (!CommandParser::parse(line.c_str(), cmd)) {
            continue;   // unbekannte Zeile ignorieren
        }

        switch (cmd.type) {
            case Command::Type::DELAY:
                scriptWaitMs = cmd.delayMs;
                scriptWaitStart = now;
                scriptWait = ScriptWait::DELAY;
                return;
            case Command::Type::WAIT:
                scriptWait = ScriptWait::ARRIVAL;
                return;
            case Command::Type::MOVE:
                doMoveCommand(cmd.x, cmd.y, cmd.snap);
                scriptWait = ScriptWait::ARRIVAL;   // Bewegung abwarten
                return;
            case Command::Type::PEEK:
                doPeekCommand(cmd.peek);
                scriptWait = ScriptWait::ARRIVAL;
                return;
            case Command::Type::RAW:
                doRawCommand(cmd.rawL, cmd.rawR);
                scriptWait = ScriptWait::ARRIVAL;
                return;
            default:
                continue;
        }
    }

    // Ende erreicht -> von vorne (Schleife).
    scriptIndex = 0;
}

void handleLine(const char* line) {
    // Beim ersten Befehl nach einem Boot die Reset-Diagnose automatisch melden,
    // damit sie im Test-GUI sichtbar wird, ohne einen Monitor anschliessen zu muessen.
    static bool sentResetLog = false;
    if (!sentResetLog) {
        sentResetLog = true;
        ble.sendLine(resetLog.format().c_str());
    }

    // Script-Upload: zwischen SCRIPT_CLEAR und SCRIPT_END werden Zeilen gesammelt.
    if (scriptLoading) {
        if (std::strcmp(line, "SCRIPT_END") == 0) {
            scriptLoading = false;
            char buf[48];
            snprintf(buf, sizeof(buf), "SCRIPT loaded %u", (unsigned)scriptLines.size());
            ble.sendLine(buf);
        } else if (scriptLines.size() < SCRIPT_MAX_LINES) {
            scriptLines.push_back(line);
        }
        return;
    }
    if (std::strcmp(line, "SCRIPT_CLEAR") == 0) {
        scriptLines.clear();
        scriptActive = false;
        scriptLoading = true;
        ble.sendLine("SCRIPT loading");
        return;
    }

    Command cmd;
    if (!CommandParser::parse(line, cmd)) {
        ble.sendLine("ERR unknown");
        return;
    }

    switch (cmd.type) {
        case Command::Type::MOVE: {
            scriptActive = false;
            doMoveCommand(cmd.x, cmd.y, cmd.snap);
            ble.sendLine("OK");
            break;
        }

        case Command::Type::PEEK:
            scriptActive = false;
            doPeekCommand(cmd.peek);
            ble.sendLine("OK");
            break;

        case Command::Type::STOP:
            autoActive = false;
            randomActive = false;
            playbackActive = false;
            recActive = false;
            scriptActive = false;
            setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
            ble.sendLine("OK");
            break;

        case Command::Type::AUTO:
            startAutoMode();
            ble.sendLine("OK");
            break;

        case Command::Type::RANDOM:
            if (randomActive) {
                randomActive = false;
                setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);   // beenden -> verstecken
                ble.sendLine("RANDOM off");
            } else {
                startRandomMode();
                ble.sendLine("RANDOM on");
            }
            break;

        case Command::Type::REC:
            if (recActive) {
                recActive = false;
                char buf[48];
                snprintf(buf, sizeof(buf), "REC off %u", (unsigned)recCount);
                ble.sendLine(buf);
            } else {
                startRecording();
                ble.sendLine("REC on");
            }
            break;

        case Command::Type::PLAY:
            if (playbackActive) {
                playbackActive = false;
                ble.sendLine("PLAY off");
            } else if (recCount == 0) {
                ble.sendLine("PLAY empty");
            } else {
                startPlayback();
                ble.sendLine("PLAY on");
            }
            break;

        case Command::Type::RUN:
            if (scriptActive) {
                scriptActive = false;
                ble.sendLine("RUN off");
            } else if (scriptLines.empty()) {
                ble.sendLine("RUN empty");
            } else {
                startScript();
                ble.sendLine("RUN on");
            }
            break;

        case Command::Type::RAW:
            // Direkter Servo-Test: IK/Planner umgehen, Pulsbreiten direkt setzen.
            scriptActive = false;
            doRawCommand(cmd.rawL, cmd.rawR);
            ble.sendLine("OK");
            break;

        case Command::Type::STATUS: {
            if (mapSending) {
                break;   // Karte wird gesendet -> keine Status-Antwort (Interleaving vermeiden)
            }
            char buf[80];
            snprintf(buf, sizeof(buf), "POS %.1f %.1f %s %u %u",
                     planner.x(), planner.y(),
                     planner.isMoving() ? "moving" : "idle",
                     (unsigned)lastPulseL, (unsigned)lastPulseR);
            ble.sendLine(buf);
            break;
        }

        case Command::Type::MAP:
            startMapSend();
            break;

        case Command::Type::RESETLOG:
            ble.sendLine(resetLog.format().c_str());
            break;

        case Command::Type::RAMP: {
            if (cmd.ramp >= 0.0f) {
                servoRampUsPerS = cmd.ramp;
            }
            char buf[48];
            snprintf(buf, sizeof(buf), "RAMP %.0f", (double)servoRampUsPerS);
            ble.sendLine(buf);
            break;
        }

        case Command::Type::DELAY:
        case Command::Type::WAIT:
            // Nur innerhalb eines Scripts sinnvoll; einzeln gesendet ignorieren wir sie.
            ble.sendLine("OK");
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
void setup() {
    resetLog.recordBoot();   // Reset-Grund persistent in NVS aufzeichnen (als Erstes)

    Serial.begin(115200);
    Serial.printf("Reset-Grund: %s (boot=%u)\n",
                  resetLog.lastReasonText().c_str(), resetLog.bootCount());

    // Servo-Kalibrierung (am Modell verifiziert):
    // "Nach innen" (mechanischer Anschlag am jeweils anderen Servo):
    //   links = 500 us, rechts = 2500 us.
    // Kurbel senkrecht nach oben (theta = 90°):
    //   links = 960 us, rechts = 2050 us.
    // Daraus folgt: Servo-Mitte (1500 us) entspricht theta = +162.90° (links)
    // bzw. theta = +15.75° (rechts). Keine Invertierung.
    servos.setCalibrationLeft(Kinematics::degToRad(162.90f), false);
    servos.setCalibrationRight(Kinematics::degToRad(15.75f), false);

    const bool ledcL = ledcAttach(PIN_SERVO_LEFT,  SERVO_FREQ_HZ, SERVO_RES_BITS);
    const bool ledcR = ledcAttach(PIN_SERVO_RIGHT, SERVO_FREQ_HZ, SERVO_RES_BITS);
    Serial.printf("LEDC attach: L=%d R=%d\n", ledcL, ledcR);

    // Arbeitsraum-Karte berechnen (fuer Tap-to-Move und Pfadplanung)
    workspace.generate();
    Serial.printf("Workspace: %d x %d\n", workspace.width(), workspace.height());

    // Startposition: versteckt (direkte Servo-Position)
    planner.setPosition(0.0f, 170.0f);   // Planner-Referenz fuer nachfolgende IK-Bewegungen
    setServoInstant(HIDE_PULSE_L, HIDE_PULSE_R);

    // BLE starten
    ble.setLineCallback(handleLine);
    ble.begin("GrabGeist");

    Serial.println("GrabGeist bereit (BLE).");
}

void loop() {
    // 50-Hz-Schleife (nicht-blockierend)
    static uint32_t prev = millis();
    const uint32_t now = millis();
    if (now - prev < LOOP_INTERVAL_MS) {
        return;
    }
    const float dt = (now - prev) / 1000.0f;
    prev = now;

    if (autoActive) {
        updateAutoMode();
    } else if (randomActive) {
        updateRandomMode();
    } else if (playbackActive) {
        updatePlayback();
    } else {
        if (scriptActive) {
            updateScript();
        }
        planner.update(dt);
        if (!directMode) {
            moveToPosition(planner.x(), planner.y());
        }
    }

    // Aufnahme (passiv): aktuelles Servo-Ziel in festem Takt abtasten.
    updateRecording();

    // Servo-Ausgabe mit begrenzter Geschwindigkeit auf das Ziel fahren.
    updateServoRamp(dt);

    // Karten-Chunks senden (nicht-blockierend).
    updateMapSend();

    // Nach abgeschlossenem Positionswechsel die neue Servo-Position melden.
    static uint16_t prevPulseL = 0, prevPulseR = 0;
    static bool     settled = false;
    if (lastPulseL != prevPulseL || lastPulseR != prevPulseR) {
        prevPulseL = lastPulseL;
        prevPulseR = lastPulseR;
        settled = false;
    } else if (!settled) {
        settled = true;
        sendPositionUpdate();
    }

    // Monitor-Ausgabe: 1x pro Sekunde die tatsaechlich an die Pins ausgegebenen
    // PWM-Pulsbreiten anzeigen (nicht in jedem 20-ms-Zyklus, damit der Monitor
    // nicht ueberlaeuft).
    static uint32_t lastLog = 0;
    if (now - lastLog >= 1000) {
        lastLog = now;
        char buf[96];
        if (lastIkValid) {
            snprintf(buf, sizeof(buf), "PWM L=%uus R=%uus  readL=%u readR=%u  pos=(%.1f, %.1f)",
                     (unsigned)lastPulseL, (unsigned)lastPulseR,
                     (unsigned)ledcRead(PIN_SERVO_LEFT), (unsigned)ledcRead(PIN_SERVO_RIGHT),
                     planner.x(), planner.y());
        } else {
            snprintf(buf, sizeof(buf), "IK ungueltig  pos=(%.1f, %.1f)",
                     planner.x(), planner.y());
        }
        Serial.println(buf);
    }
}
