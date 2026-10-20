// main.cpp — HostageTarget Firmware (ESP32-S3)
//
// Test-Steuerung ueber BLE (Handy):
//   BLE -> CommandParser -> MotionPlanner -> Kinematics -> ServoController -> LEDC

#include <Arduino.h>

#include <CommandParser.h>
#include <Kinematics.h>
#include <MotionPlanner.h>
#include <ServoController.h>

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

// Zuletzt an die Servo-Pins ausgegebene PWM-Pulsbreiten (fuer die Monitor-Ausgabe).
static uint16_t lastPulseL = 0;
static uint16_t lastPulseR = 0;
static bool     lastIkValid = false;
static bool     directMode = false;   // true = Servos direkt ansteuern (RAW), IK/Planner pausiert

// ---------------------------------------------------------------------------
void servoWriteUs(uint8_t pin, uint16_t pulseUs) {
    const uint32_t duty = (uint32_t)pulseUs * SERVO_DUTY_FS / SERVO_PERIOD_US;
    if (!ledcWrite(pin, duty)) {
        Serial.printf("ledcWrite fehlgeschlagen (pin=%u duty=%u)\n", pin, (unsigned)duty);
    }
}

void moveToPosition(float x, float y) {
    float thetaL = 0.0f, thetaR = 0.0f;
    if (Kinematics::calculateIK(x, y, thetaL, thetaR)) {
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
// BLE-Empfang: eine Zeile -> Befehl parsen und ausfuehren.
// (Wird aus dem BLE-Kontext aufgerufen; fuer den Testbetrieb reicht das.)
// ---------------------------------------------------------------------------
void handleLine(const char* line) {
    Command cmd;
    if (!CommandParser::parse(line, cmd)) {
        ble.sendLine("ERR unknown");
        return;
    }

    switch (cmd.type) {
        case Command::Type::MOVE:
            directMode = false;
            planner.moveTo(cmd.x, cmd.y,
                           cmd.snap ? MotionPlanner::MoveType::SNAP
                                    : MotionPlanner::MoveType::SMOOTH);
            ble.sendLine("OK");
            break;

        case Command::Type::PEEK:
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
            setServoDirect(HIDE_PULSE_L, HIDE_PULSE_R);
            ble.sendLine("OK");
            break;

        case Command::Type::RAW:
            // Direkter Servo-Test: IK/Planner umgehen, Pulsbreiten direkt setzen.
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

    planner.update(dt);
    if (!directMode) {
        moveToPosition(planner.x(), planner.y());
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
