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
// PWM (LEDC): 50 Hz, 16 Bit, Pulsbreite 500..2500 us.
// ---------------------------------------------------------------------------
static const uint32_t SERVO_FREQ_HZ   = 50;
static const uint8_t  SERVO_RES_BITS  = 16;
static const uint32_t SERVO_PERIOD_US = 1000000u / SERVO_FREQ_HZ;   // 20000 us
static const uint32_t SERVO_DUTY_FS   = (1u << SERVO_RES_BITS);     // 2^16

static const uint32_t LOOP_INTERVAL_MS = 20;   // 50 Hz

// ---------------------------------------------------------------------------
// Preset-Koordinaten (ANPASSEN, sobald die Mechanik aus Fusion steht!)
// ---------------------------------------------------------------------------
static const float POS_HIDE_X  = 0.0f,  POS_HIDE_Y  = 170.0f;
static const float POS_LEFT_X  = -50.0f, POS_LEFT_Y  = 150.0f;
static const float POS_RIGHT_X = 50.0f,  POS_RIGHT_Y = 150.0f;
static const float POS_TOP_X   = 0.0f,  POS_TOP_Y   = 190.0f;

// Software-Bausteine
static MotionPlanner   planner;
static ServoController servos;
static BleUart         ble;

// ---------------------------------------------------------------------------
void servoWriteUs(uint8_t pin, uint16_t pulseUs) {
    const uint32_t duty = (uint32_t)pulseUs * SERVO_DUTY_FS / SERVO_PERIOD_US;
    ledcWrite(pin, duty);
}

void moveToPosition(float x, float y) {
    float thetaL = 0.0f, thetaR = 0.0f;
    if (Kinematics::calculateIK(x, y, thetaL, thetaR)) {
        servoWriteUs(PIN_SERVO_LEFT,  servos.angleToPulseUsLeft(thetaL));
        servoWriteUs(PIN_SERVO_RIGHT, servos.angleToPulseUsRight(thetaR));
    }
    // ungueltige Position -> Servos halten die letzte Stellung
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
            planner.moveTo(cmd.x, cmd.y,
                           cmd.snap ? MotionPlanner::MoveType::SNAP
                                    : MotionPlanner::MoveType::SMOOTH);
            ble.sendLine("OK");
            break;

        case Command::Type::PEEK:
            switch (cmd.peek) {
                case Command::Peek::LEFT:
                    planner.moveTo(POS_LEFT_X, POS_LEFT_Y, MotionPlanner::MoveType::SMOOTH);
                    break;
                case Command::Peek::RIGHT:
                    planner.moveTo(POS_RIGHT_X, POS_RIGHT_Y, MotionPlanner::MoveType::SMOOTH);
                    break;
                case Command::Peek::TOP:
                    planner.moveTo(POS_TOP_X, POS_TOP_Y, MotionPlanner::MoveType::SMOOTH);
                    break;
                case Command::Peek::HIDE:
                    planner.moveTo(POS_HIDE_X, POS_HIDE_Y, MotionPlanner::MoveType::SNAP);
                    break;
            }
            ble.sendLine("OK");
            break;

        case Command::Type::STOP:
            planner.moveTo(POS_HIDE_X, POS_HIDE_Y, MotionPlanner::MoveType::SNAP);
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

    // Servo-Kalibrierung (ANPASSEN!)
    // servos.setCalibrationLeft(Kinematics::degToRad(0.0f),  false);
    // servos.setCalibrationRight(Kinematics::degToRad(0.0f), true);

    ledcAttach(PIN_SERVO_LEFT,  SERVO_FREQ_HZ, SERVO_RES_BITS);
    ledcAttach(PIN_SERVO_RIGHT, SERVO_FREQ_HZ, SERVO_RES_BITS);

    // Startposition: versteckt
    planner.setPosition(POS_HIDE_X, POS_HIDE_Y);
    moveToPosition(planner.x(), planner.y());

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
    moveToPosition(planner.x(), planner.y());
}
