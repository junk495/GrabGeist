// main.cpp — HostageTarget Firmware (ESP32-S3)
//
// Test-Steuerung ueber BLE (Handy):
//   BLE -> CommandParser -> MotionPlanner -> Kinematics -> ServoController -> LEDC

#include <Arduino.h>

#include <CommandParser.h>
#include <Kinematics.h>
#include <MotionPlanner.h>
#include <ServoController.h>
#include <Workspace.h>

#include <string>

#include "BleUart.h"

// ---------------------------------------------------------------------------
// Pinbelegung (ANNAHME — an deinen Aufbau anpassen!)
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
// Preset-Koordinaten (ANPASSEN, sobald die Mechanik aus Fusion steht!)
// ---------------------------------------------------------------------------
static const float POS_LEFT_X  = -50.0f, POS_LEFT_Y  = 150.0f;
static const float POS_RIGHT_X = 50.0f,  POS_RIGHT_Y = 150.0f;

// "Versteckt" als direkte Servo-Position (am Aufbau ermittelt).
// Kein IK-Punkt: die Kurbeln stehen dabei weit auseinander (Schubstangen
// treffen sich nicht) -> Ziel liegt tief hinter dem Pfosten.
static const uint16_t HIDE_PULSE_L = 1660;
static const uint16_t HIDE_PULSE_R = 1350;

// Software-Bausteine
static MotionPlanner   planner;
static ServoController servos;
static BleUart         ble;
static Workspace       workspace;

// Zuletzt an die Servo-Pins ausgegebene PWM-Pulsbreiten (fuer die Monitor-Ausgabe).
static uint16_t lastPulseL = 0;
static uint16_t lastPulseR = 0;
static bool     lastIkValid = false;
static bool     directMode = false;   // true = Servos direkt ansteuern (RAW), IK/Planner pausiert

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
void servoWriteUs(uint8_t pin, uint16_t pulseUs) {
    const uint32_t duty = (uint32_t)pulseUs * SERVO_DUTY_FS / SERVO_PERIOD_US;
    if (!ledcWrite(pin, duty)) {
        Serial.printf("ledcWrite fehlgeschlagen (pin=%u duty=%u)\n", pin, (unsigned)duty);
    }
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
        lastPulseL = servos.angleToPulseUsLeft(thetaL);
        lastPulseR = servos.angleToPulseUsRight(thetaR);
        lastIkValid = true;
        servoWriteUs(PIN_SERVO_LEFT,  lastPulseL);
        servoWriteUs(PIN_SERVO_RIGHT, lastPulseR);
    } else {
        lastIkValid = false;   // ungueltige Position -> Servos halten letzte Stellung
    }
}

// ---------------------------------------------------------------------------
// Servos direkt auf feste Pulsbreiten setzen (umgeht IK/Planner).
// ---------------------------------------------------------------------------
void setServoDirect(uint16_t pulseL, uint16_t pulseR) {
    directMode = true;
    lastPulseL = pulseL;
    lastPulseR = pulseR;
    lastIkValid = true;
    servoWriteUs(PIN_SERVO_LEFT,  pulseL);
    servoWriteUs(PIN_SERVO_RIGHT, pulseR);
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
        servoWriteUs(PIN_SERVO_LEFT,  l);
        servoWriteUs(PIN_SERVO_RIGHT, r);
        lastPulseL = l;
        lastPulseR = r;
        lastIkValid = true;

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

void handleLine(const char* line) {
    Command cmd;
    if (!CommandParser::parse(line, cmd)) {
        ble.sendLine("ERR unknown");
        return;
    }

    switch (cmd.type) {
        case Command::Type::MOVE: {
            autoActive = false;
            directMode = false;

            // Ziel auf den naechsten gueltigen Punkt snappen
            float tx = cmd.x, ty = cmd.y;
            workspace.nearestValid(cmd.x, cmd.y, tx, ty);

            if (cmd.snap) {
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
            ble.sendLine("OK");
            break;
        }

        case Command::Type::PEEK:
            autoActive = false;
            directMode = false;
            switch (cmd.peek) {
                case Command::Peek::LEFT:
                    planner.moveTo(POS_LEFT_X, POS_LEFT_Y, MotionPlanner::MoveType::SMOOTH);
                    break;
                case Command::Peek::RIGHT:
                    planner.moveTo(POS_RIGHT_X, POS_RIGHT_Y, MotionPlanner::MoveType::SMOOTH);
                    break;
                case Command::Peek::TOP:
                    // Referenz: beide Kurbeln senkrecht nach oben (theta = 90°).
                    // Montage-Referenz; umgeht die IK, da das exakte "beide oben"
                    // knapp ausserhalb der IK-Reichweite liegt.
                    setServoDirect(servos.angleToPulseUsLeft(Kinematics::degToRad(90.0f)),
                                   servos.angleToPulseUsRight(Kinematics::degToRad(90.0f)));
                    break;
                case Command::Peek::HIDE:
                    setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
                    break;
            }
            ble.sendLine("OK");
            break;

        case Command::Type::STOP:
            autoActive = false;
            setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
            ble.sendLine("OK");
            break;

        case Command::Type::AUTO:
            startAutoMode();
            ble.sendLine("OK");
            break;

        case Command::Type::RAW:
            // Direkter Servo-Test: IK/Planner umgehen, Pulsbreiten direkt setzen.
            autoActive = false;
            setServoDirect(cmd.rawL, cmd.rawR);
            ble.sendLine("OK");
            break;

        case Command::Type::STATUS: {
            char buf[64];
            snprintf(buf, sizeof(buf), "POS %.1f %.1f %s",
                     planner.x(), planner.y(),
                     planner.isMoving() ? "moving" : "idle");
            ble.sendLine(buf);
            break;
        }

        case Command::Type::MAP:
            sendMap();
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Servo-Kalibrierung (am Aufbau ermittelt):
    // "Nach innen" (mechanischer Anschlag am jeweils anderen Servo):
    //   links = 500 us, rechts = 2500 us.
    // Kurbel senkrecht nach oben (theta = 90°):
    //   links = 960 us, rechts = 2010 us.
    // Daraus folgt: Servo-Mitte (1500 us) entspricht theta = +162.90° (links)
    // bzw. theta = +21.15° (rechts). Keine Invertierung.
    servos.setCalibrationLeft(Kinematics::degToRad(162.90f), false);
    servos.setCalibrationRight(Kinematics::degToRad(21.15f), false);

    const bool ledcL = ledcAttach(PIN_SERVO_LEFT,  SERVO_FREQ_HZ, SERVO_RES_BITS);
    const bool ledcR = ledcAttach(PIN_SERVO_RIGHT, SERVO_FREQ_HZ, SERVO_RES_BITS);
    Serial.printf("LEDC attach: L=%d R=%d\n", ledcL, ledcR);

    // Arbeitsraum-Karte berechnen (fuer Tap-to-Move und Pfadplanung)
    workspace.generate();
    Serial.printf("Workspace: %d x %d\n", workspace.width(), workspace.height());

    // Startposition: versteckt (direkte Servo-Position)
    planner.setPosition(0.0f, 170.0f);   // Planner-Referenz fuer nachfolgende IK-Bewegungen
    setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);

    // BLE starten
    ble.setLineCallback(handleLine);
    ble.begin("HostageTarget");

    Serial.println("HostageTarget bereit (BLE).");
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
    } else {
        planner.update(dt);
        if (!directMode) {
            moveToPosition(planner.x(), planner.y());
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
                     planner.x(), planner.y());
        } else {
            snprintf(buf, sizeof(buf), "IK ungueltig  pos=(%.1f, %.1f)",
                     planner.x(), planner.y());
        }
        Serial.println(buf);
    }
}
