// main.cpp — GrabGeist Firmware (ESP32-S3)
//
// Test-Steuerung ueber BLE (Handy):
//   BLE -> CommandParser -> Kinematics -> ServoController -> LEDC
//   (Bewegung: Wegpunkt-Folge + ServoSpeed-Rampe)

#include <Arduino.h>
#include <esp_random.h>   // esp_random() fuer den Zufallsmodus
#include <Preferences.h>
#include <LittleFS.h>
#include <dirent.h>

#include <ArduinoJson.h>
#include <CommandParser.h>
#include <Kinematics.h>
#include <Sequence.h>
#include <ServoController.h>
#include <ServoSpeed.h>
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
// Strombegrenzung (intern): hoechste erlaubte Puls-Slew-Rate [us/s].
// Sicherheits-Deckel gegen Stromspitzen/Brownout; wird dauerhaft in NVS gespeichert.
// Befehl "RAMP <us/s>" setzt den Wert ("RAMP" fragt ab).
// ---------------------------------------------------------------------------
static float maxRateUsPerSec = ServoSpeed::DEFAULT_LIMIT_US_PER_S;

// Geschwindigkeit der aktuellen Bewegung: 0..100 % der Strombegrenzung.
// 100 % = volle Strombegrenzung; 50 % = halb so schnell.
// Befehl "SPEED <pct>" setzt den Standardwert ("SPEED" fragt ab).
static float servoSpeedPercent = 100.0f;

// ---------------------------------------------------------------------------
// Strombegrenzung dauerhaft speichern/laden (NVS).
// ---------------------------------------------------------------------------
static const char* kPrefsNamespace = "grabgeist";

static void loadMaxRate() {
    Preferences prefs;
    prefs.begin(kPrefsNamespace, true);
    maxRateUsPerSec = prefs.getFloat("maxRate", ServoSpeed::DEFAULT_LIMIT_US_PER_S);
    prefs.end();
    maxRateUsPerSec = ServoSpeed::clampRate(maxRateUsPerSec);
}

static void saveMaxRate() {
    Preferences prefs;
    prefs.begin(kPrefsNamespace, false);
    prefs.putFloat("maxRate", maxRateUsPerSec);
    prefs.end();
}

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
static ServoController servos;
static BleUart         ble;
static Workspace       workspace;
static ResetLog        resetLog;

// Wegpunkt-Folge (X/Y), abgearbeitet nach Ankunft der Rampe:
// Tempo = Strombegrenzung * speed%.
static std::vector<Point> wpPath;
static size_t   wpIndex = 0;
static bool     wpActive = false;

// Letzte IK-Zielposition (Startpunkt fuer den naechsten A*-Pfad).
static float currentX = 0.0f;
static float currentY = 170.0f;

// Zuletzt an die Servo-Pins ausgegebene PWM-Pulsbreiten (fuer die Monitor-Ausgabe).
static uint16_t lastPulseL = 0;
static uint16_t lastPulseR = 0;
static bool     lastIkValid = false;
static bool     directMode = false;   // true = Servos direkt ansteuern (RAW), IK/Planner pausiert

// Servo-Soll (Ziel) und tatsaechlich ausgegebene Pulsbreite (durch die Rampe begrenzt).
static uint16_t desiredPulseL = 0, desiredPulseR = 0;
static uint16_t outputPulseL = 0, outputPulseR = 0;

// Zuletzt gueltige Kurbelwinkel (Hysterese-Hint fuer die bimodale IK).
// Wird in setTargetPulses() mit der tatsaechlichen Servo-Konfiguration synchron
// gehalten, damit die naechste IK den richtigen Ast waehlt.
static float lastThetaL = 0.0f;
static float lastThetaR = 0.0f;

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
// JSON-Ablauf (Sequence) - Zustand.
// ---------------------------------------------------------------------------
enum class SeqWait { NONE, DELAY, ARRIVAL };

static Sequence currentSequence;
static bool     seqActive = false;
static size_t   seqIndex = 0;
static SeqWait  seqWait = SeqWait::NONE;
static uint32_t seqWaitStart = 0;
static uint32_t seqWaitMs = 0;

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
    // Hysterese-Hint auf die tatsaechliche Servo-Konfiguration setzen. Sonst
    // waehlt die IK beim ersten MOVE nach dem Boot (Hint 0/0) den Over-Center-Ast
    // und der linke Servo ueberdreht ueber die Senkrechte hinaus.
    lastThetaL = servos.pulseToAngleUsLeft((float)pulseL);
    lastThetaR = servos.pulseToAngleUsRight((float)pulseR);
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
// pro Zeitschritt maximal um (Strombegrenzung * speed%) * dt in Richtung Ziel.
// Die Strombegrenzung ist der Sicherheits-Deckel; speed% skaliert darunter.
// ---------------------------------------------------------------------------
void updateServoRamp(float dt) {
    const float rate = ServoSpeed::rateForPercent(maxRateUsPerSec, servoSpeedPercent);
    if (rate <= 0.0f) {
        return;   // Geschwindigkeit 0 % -> halten
    }

    const float maxStep = rate * dt;
    const float dl = (float)desiredPulseL - (float)outputPulseL;
    const float dr = (float)desiredPulseR - (float)outputPulseR;
    if (dl > maxStep)       outputPulseL = (uint16_t)((float)outputPulseL + maxStep);
    else if (dl < -maxStep) outputPulseL = (uint16_t)((float)outputPulseL - maxStep);
    else                    outputPulseL = desiredPulseL;
    if (dr > maxStep)       outputPulseR = (uint16_t)((float)outputPulseR + maxStep);
    else if (dr < -maxStep) outputPulseR = (uint16_t)((float)outputPulseR - maxStep);
    else                    outputPulseR = desiredPulseR;

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
    wpActive = false;
    seqActive = false;
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
    wpActive = false;
    seqActive = false;
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
    wpActive = false;
    seqActive = false;
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
    wpActive = false;
    seqActive = false;
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
void startMoveTo(float x, float y, bool snap) {
    float tx = x, ty = y;
    workspace.nearestValid(x, y, tx, ty);

    if (snap) {
        wpPath.clear();
        wpPath.push_back({tx, ty});
    } else {
        float sx = currentX, sy = currentY;
        workspace.nearestValid(sx, sy, sx, sy);
        if (!workspace.findPath(sx, sy, tx, ty, wpPath)) {
            wpPath.clear();
            wpPath.push_back({tx, ty});
        }
    }
    wpIndex = 0;
    wpActive = true;

    currentX = tx;
    currentY = ty;
}

void doMoveCommand(float x, float y, bool snap) {
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    directMode = false;
    startMoveTo(x, y, snap);
}

void doPeekCommand(Command::Peek peek) {
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    directMode = false;
    switch (peek) {
        case Command::Peek::LEFT:
            startMoveTo(POS_LEFT_X, POS_LEFT_Y, false);
            break;
        case Command::Peek::RIGHT:
            startMoveTo(POS_RIGHT_X, POS_RIGHT_Y, false);
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
// JSON-Ablauf (Sequence): gespeicherten Ablauf als Schleife abspielen.
// ---------------------------------------------------------------------------
static const char* kSequencePath = "/sequence.json";

static void saveSequence() {
    File f = LittleFS.open(kSequencePath, "w");
    if (!f) return;
    const std::string json = Sequence::toJson(currentSequence);
    f.print(json.c_str());
    f.close();
}

static void loadSequence() {
    if (!LittleFS.exists(kSequencePath)) return;
    File f = LittleFS.open(kSequencePath, "r");
    if (!f) return;
    const String s = f.readString();
    f.close();
    Sequence::fromJson(s.c_str(), currentSequence);
}

// ---------------------------------------------------------------------------
// Benannte Ablaeufe (Bibliothek): mehrere Sequenzen dauerhaft auf der ESP
// ablegen -> unabhaengig von Handy/PC. Gespeichert unter /seq/<name>.json.
// ---------------------------------------------------------------------------
static const char* kSeqDir = "/seq";

static std::string sanitizeSeqName(const std::string& name) {
    std::string out;
    for (char c : name) {
        if (c == '/' || c == '\\' || c == '\0' || c == '\n' || c == '\r') {
            out += '_';
        } else {
            out += c;
        }
    }
    if (out.empty()) {
        out = "seq";
    }
    return out;
}

static std::string seqFilePath(const std::string& name) {
    return std::string(kSeqDir) + "/" + sanitizeSeqName(name) + ".json";
}

static bool saveNamedSequence(const std::string& name) {
    currentSequence.name = name;
    File f = LittleFS.open(seqFilePath(name).c_str(), "w");
    if (!f) {
        return false;
    }
    const std::string json = Sequence::toJson(currentSequence);
    f.print(json.c_str());
    f.close();
    return true;
}

static bool loadNamedSequence(const std::string& name) {
    File f = LittleFS.open(seqFilePath(name).c_str(), "r");
    if (!f) {
        return false;
    }
    const String s = f.readString();
    f.close();
    return Sequence::fromJson(s.c_str(), currentSequence);
}

static bool deleteNamedSequence(const std::string& name) {
    return LittleFS.remove(seqFilePath(name).c_str());
}

static std::vector<std::string> listNamedSequences() {
    std::vector<std::string> names;
    const char* mp = LittleFS.mountpoint();
    const std::string dirPath = (mp ? std::string(mp) : std::string("")) + kSeqDir;
    DIR* dir = opendir(dirPath.c_str());
    if (dir == nullptr) {
        return names;
    }
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        const std::string n = entry->d_name;
        // Nur .json-Dateien zaehlen (ueber den Namen, nicht ueber d_type:
        // LittleFS liefert d_type teilweise als DT_UNKNOWN).
        if (n.size() > 5 && n.compare(n.size() - 5, 5, ".json") == 0) {
            names.push_back(n.substr(0, n.size() - 5));
        }
    }
    closedir(dir);
    return names;
}

static void sendSequenceList() {
    const std::vector<std::string> names = listNamedSequences();
    Serial.printf("SEQ_LIST: %u Ablaeufe\n", (unsigned)names.size());

    // Als EINE JSON-Zeile senden -> keine Reihenfolge-/Verlust-Probleme
    // zwischen mehreren Einzelzeilen (SEQ_NAME ... / SEQ_LIST_DONE).
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& n : names) {
        arr.add(n);
    }
    std::string json;
    serializeJson(doc, json);
    const std::string payload = "SEQ_LISTJSON " + json;
    ble.sendChunked(payload.data(), payload.size());
    ble.sendLine("");
}

static void startSequence() {
    if (currentSequence.steps.empty()) return;
    seqActive = true;
    seqIndex = 0;
    seqWait = SeqWait::NONE;
    autoActive = false;
    randomActive = false;
    playbackActive = false;
    recActive = false;
    wpActive = false;
    directMode = false;
}

static void updateSequence() {
    if (!seqActive) return;
    const uint32_t now = millis();

    if (seqWait == SeqWait::DELAY) {
        // Haltezeit laeuft noch.
        if (now - seqWaitStart < seqWaitMs) return;
        seqWait = SeqWait::NONE;
    } else if (seqWait == SeqWait::ARRIVAL) {
        // Auf Ankunft warten, danach ggf. die Haltezeit starten.
        if (wpActive || outputPulseL != desiredPulseL || outputPulseR != desiredPulseR) return;
        if (seqWaitMs > 0) {
            seqWaitStart = now;
            seqWait = SeqWait::DELAY;
            return;
        }
        seqWait = SeqWait::NONE;
    }

    if (seqIndex >= currentSequence.steps.size()) {
        if (!currentSequence.loop) {
            seqActive = false;
            servoSpeedPercent = 100.0f;   // Schritt-Geschwindigkeit nicht "kleben" lassen
            return;
        }
        seqIndex = 0;
    }

    const Sequence::Step& step = currentSequence.steps[seqIndex++];
    switch (step.type) {
        case Sequence::StepType::MOVE:
            servoSpeedPercent = step.move.speed;
            doMoveCommand(step.move.x, step.move.y, false);
            seqWaitMs = step.move.holdMs;
            seqWait = SeqWait::ARRIVAL;
            return;

        case Sequence::StepType::PRESET: {
            servoSpeedPercent = step.preset.speed;
            Command::Peek p;
            if (step.preset.name == "left")       p = Command::Peek::LEFT;
            else if (step.preset.name == "right") p = Command::Peek::RIGHT;
            else if (step.preset.name == "hide")  p = Command::Peek::HIDE;
            else return;
            doPeekCommand(p);
            seqWaitMs = step.preset.holdMs;
            seqWait = SeqWait::ARRIVAL;
            return;
        }
    }
}

void handleLine(const char* line) {
    // Beim ersten Befehl nach einem Boot die Reset-Diagnose automatisch melden,
    // damit sie im Test-GUI sichtbar wird, ohne einen Monitor anschliessen zu muessen.
    static bool sentResetLog = false;
    if (!sentResetLog) {
        sentResetLog = true;
        ble.sendLine(resetLog.format().c_str());
    }

    // Sequence-Befehle: JSON kann laenger als der CommandParser-Puffer sein,
    // deshalb direkt hier auswerten.
    if (std::strncmp(line, "SEQ_SET ", 8) == 0) {
        Sequence parsed;
        std::string err;
        if (Sequence::fromJson(line + 8, parsed, &err)) {
            currentSequence = parsed;
            saveSequence();
            ble.sendLine("SEQ saved");
        } else {
            ble.sendLine("SEQ error");
        }
        return;
    }
    if (std::strcmp(line, "SEQ_RUN") == 0) {
        if (seqActive) {
            seqActive = false;
            servoSpeedPercent = 100.0f;   // Schritt-Geschwindigkeit nicht "kleben" lassen
            setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
            ble.sendLine("SEQ off");
        } else {
            startSequence();
            ble.sendLine("SEQ run");
        }
        return;
    }
    if (std::strcmp(line, "SEQ_STOP") == 0) {
        seqActive = false;
        servoSpeedPercent = 100.0f;   // Geschwindigkeit zuruecksetzen
        setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
        ble.sendLine("SEQ stop");
        return;
    }
    if (std::strcmp(line, "SEQ_GET") == 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "SEQ_STATE loop=%d autostart=%d steps=%u",
                 currentSequence.loop ? 1 : 0,
                 currentSequence.autostart ? 1 : 0,
                 (unsigned)currentSequence.steps.size());
        ble.sendLine(buf);
        return;
    }
    if (std::strncmp(line, "SEQ_SAVE ", 9) == 0) {
        const char* name = line + 9;
        if (*name == '\0') {
            ble.sendLine("SEQ_ERR empty");
        } else if (saveNamedSequence(name)) {
            ble.sendLine("SEQ_SAVED");
        } else {
            ble.sendLine("SEQ_ERR write");
        }
        return;
    }
    if (std::strcmp(line, "SEQ_LIST") == 0) {
        sendSequenceList();
        return;
    }
    if (std::strncmp(line, "SEQ_LOAD ", 9) == 0) {
        const char* name = line + 9;
        if (loadNamedSequence(name)) {
            const std::string json = Sequence::toJsonCompact(currentSequence);
            const std::string payload = "SEQ_JSON " + json;
            ble.sendChunked(payload.data(), payload.size());
            ble.sendLine("");
        } else {
            ble.sendLine("SEQ_ERR notfound");
        }
        return;
    }
    if (std::strncmp(line, "SEQ_DELETE ", 11) == 0) {
        const char* name = line + 11;
        if (deleteNamedSequence(name)) {
            ble.sendLine("SEQ_DELETED");
        } else {
            ble.sendLine("SEQ_ERR notfound");
        }
        return;
    }

    Command cmd;
    if (!CommandParser::parse(line, cmd)) {
        ble.sendLine("ERR unknown");
        return;
    }

    switch (cmd.type) {
        case Command::Type::MOVE: {
            seqActive = false;
            doMoveCommand(cmd.x, cmd.y, cmd.snap);
            ble.sendLine("OK");
            break;
        }

        case Command::Type::PEEK:
            seqActive = false;
            doPeekCommand(cmd.peek);
            ble.sendLine("OK");
            break;

        case Command::Type::STOP:
            autoActive = false;
            randomActive = false;
            playbackActive = false;
            recActive = false;
            seqActive = false;
            servoSpeedPercent = 100.0f;   // Geschwindigkeit zuruecksetzen
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

        case Command::Type::RAW:
            // Direkter Servo-Test: IK/Planner umgehen, Pulsbreiten direkt setzen.
            seqActive = false;
            doRawCommand(cmd.rawL, cmd.rawR);
            ble.sendLine("OK");
            break;

        case Command::Type::STATUS: {
            if (mapSending) {
                break;   // Karte wird gesendet -> keine Status-Antwort (Interleaving vermeiden)
            }
            char buf[80];
            snprintf(buf, sizeof(buf), "POS %.1f %.1f %s %u %u",
                     currentX, currentY,
                     wpActive ? "moving" : "idle",
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
                maxRateUsPerSec = ServoSpeed::clampRate(cmd.ramp);
                saveMaxRate();   // dauerhaft speichern
            }
            char buf[48];
            snprintf(buf, sizeof(buf), "RAMP %.0f", (double)maxRateUsPerSec);
            ble.sendLine(buf);
            break;
        }

        case Command::Type::SPEED: {
            if (cmd.speed >= 0.0f) {
                servoSpeedPercent = ServoSpeed::clampPercent(cmd.speed);
            }
            char buf[48];
            snprintf(buf, sizeof(buf), "SPEED %.0f", (double)servoSpeedPercent);
            ble.sendLine(buf);
            break;
        }

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
void setup() {
    resetLog.recordBoot();   // Reset-Grund persistent in NVS aufzeichnen (als Erstes)
    loadMaxRate();           // Strombegrenzung aus NVS laden

    Serial.begin(115200);
    Serial.printf("Reset-Grund: %s (boot=%u)\n",
                  resetLog.lastReasonText().c_str(), resetLog.bootCount());

    // Web-UI-Dateisystem mounten (ggf. beim ersten Mal formatieren).
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount fehlgeschlagen");
    }
    LittleFS.mkdir(kSeqDir);   // Ordner fuer benannte Ablaeufe anlegen
    loadSequence();

    // Autostart: Ablauf ohne Handy-Eingriff starten (auch nach Reset).
    if (currentSequence.autostart && !currentSequence.steps.empty()) {
        startSequence();
    }

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

    // Sofort eine gueltige Pulsbreite ausgeben: LEDC startet mit Duty 0 (= 0 us
    // Pulsbreite, unter dem Servo-Minimum von 500 us). Waehrend der folgenden
    // workspace.generate() wuerden die Servos sonst an den mechanischen Anschlag
    // fahren (linker Servo "nach innen" -> verklemmt).
    setServoInstant(HIDE_PULSE_L, HIDE_PULSE_R);

    // Arbeitsraum-Karte berechnen (fuer Tap-to-Move und Pfadplanung)
    workspace.generate();
    Serial.printf("Workspace: %d x %d\n", workspace.width(), workspace.height());

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
        if (seqActive) {
            updateSequence();
        }
        if (wpActive && !directMode) {
            moveToPosition(wpPath[wpIndex].x, wpPath[wpIndex].y);
        }
    }

    // Aufnahme (passiv): aktuelles Servo-Ziel in festem Takt abtasten.
    updateRecording();

    // Servo-Ausgabe mit begrenzter Geschwindigkeit auf das Ziel fahren.
    updateServoRamp(dt);

    // Wegpunkt-Fortschritt: erst wenn die Rampe den aktuellen Punkt erreicht hat.
    if (wpActive && outputPulseL == desiredPulseL && outputPulseR == desiredPulseR) {
        if (wpIndex + 1 < wpPath.size()) {
            ++wpIndex;
        } else {
            wpActive = false;
        }
    }

    // Karten-Chunks senden (nicht-blockierend).
    updateMapSend();

    // Nach abgeschlossenem Positionswechsel die neue Servo-Position melden.
    static uint16_t prevPulseL = 0, prevPulseR = 0;
    static bool     settled = false;
    if (!wpActive) {
        if (lastPulseL != prevPulseL || lastPulseR != prevPulseR) {
            prevPulseL = lastPulseL;
            prevPulseR = lastPulseR;
            settled = false;
        } else if (!settled) {
            settled = true;
            sendPositionUpdate();
        }
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
                     currentX, currentY);
        } else {
            snprintf(buf, sizeof(buf), "IK ungueltig  pos=(%.1f, %.1f)",
                     currentX, currentY);
        }
        Serial.println(buf);
    }
}
