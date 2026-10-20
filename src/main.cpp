// main.cpp — HostageTarget Firmware (ESP32-S3)
//
// Verdrahtet die Software-Bausteine:
//   MotionPlanner (Bahn) -> Kinematics (IK) -> ServoController (Winkel->PWM) -> LEDC (Servos)
//
// Update-Rate: 50 Hz (20 ms), nicht-blockierend.

#include <Arduino.h>

#include <Kinematics.h>
#include <MotionPlanner.h>
#include <ServoController.h>

// ---------------------------------------------------------------------------
// Pinbelegung (ANNAHME — an deinen Aufbau anpassen!)
//
// Gewaehlt wurden GPIOs OHNE Konflikt mit Boot-Strapping-/Flash-/PSRAM-/
// UART0-/USB-Belegung des ESP32-S3:
//   Boot-Strapping (nicht nutzen):  GPIO0, GPIO3, GPIO45, GPIO46
//   UART0 / Serial-Konsole:         GPIO43 (TX), GPIO44 (RX)
//   Native USB (falls genutzt):     GPIO19 (D-), GPIO20 (D+)
//   Flash / PSRAM (intern belegt):  GPIO26..32 (Flash), GPIO33..37 (octal PSRAM)
// ---------------------------------------------------------------------------
static const uint8_t PIN_SERVO_LEFT  = 4;   // linker Servo  (PWM)
static const uint8_t PIN_SERVO_RIGHT = 5;   // rechter Servo (PWM)

// ---------------------------------------------------------------------------
// PWM (LEDC) fuer 270-Grad-RC-Servos: 50 Hz, 14 Bit.
// Hinweis: ESP32-S3-LEDC erlaubt maximal 14 Bit Aufloesung.
// Pulsbreite 500..2500 us wird auf Duty 0..2^14 abgebildet.
// ---------------------------------------------------------------------------
static const uint32_t SERVO_FREQ_HZ   = 50;
static const uint8_t  SERVO_RES_BITS  = 14;
static const uint32_t SERVO_PERIOD_US = 1000000u / SERVO_FREQ_HZ;   // 20000 us
static const uint32_t SERVO_DUTY_FS   = (1u << SERVO_RES_BITS);     // 2^14 = 16384

// Update-Schleife
static const uint32_t LOOP_INTERVAL_MS = 20;   // 50 Hz

// Software-Bausteine
static MotionPlanner   planner;
static ServoController servos;

// ---------------------------------------------------------------------------
// Pulsbreite [us] -> LEDC-Duty und an den Pin schreiben.
// ---------------------------------------------------------------------------
void servoWriteUs(uint8_t pin, uint16_t pulseUs) {
    const uint32_t duty = (uint32_t)pulseUs * SERVO_DUTY_FS / SERVO_PERIOD_US;
    ledcWrite(pin, duty);
}

// ---------------------------------------------------------------------------
// Zielposition (x, y) anfahren: IK -> Winkel -> Puls -> Servos.
// Ungueltige Positionen werden ignoriert (Servos halten die letzte Stellung).
// ---------------------------------------------------------------------------
void moveToPosition(float x, float y) {
    float thetaL = 0.0f, thetaR = 0.0f;
    if (Kinematics::calculateIK(x, y, thetaL, thetaR)) {
        servoWriteUs(PIN_SERVO_LEFT,  servos.angleToPulseUsLeft(thetaL));
        servoWriteUs(PIN_SERVO_RIGHT, servos.angleToPulseUsRight(thetaR));
    }
}

void setup() {
    Serial.begin(115200);

    // Servo-Kalibrierung (Nullpunkt-Offset [rad] + Richtung) — ANPASSEN!
    // Beispiel (Werte am Aufbau ermitteln):
    //   servos.setCalibrationLeft(Kinematics::degToRad(0.0f),  false);
    //   servos.setCalibrationRight(Kinematics::degToRad(0.0f), true);

    // LEDC fuer beide Servos initialisieren
    ledcAttach(PIN_SERVO_LEFT,  SERVO_FREQ_HZ, SERVO_RES_BITS);
    ledcAttach(PIN_SERVO_RIGHT, SERVO_FREQ_HZ, SERVO_RES_BITS);

    // Startposition: Ziel mittig/oben (hinter dem Pfosten "versteckt")
    planner.setPosition(0.0f, 170.0f);
    moveToPosition(planner.x(), planner.y());

    Serial.println("HostageTarget bereit.");
}

void loop() {
    // Feste 50-Hz-Schleife (nicht-blockierend)
    static uint32_t prev = millis();
    const uint32_t now = millis();
    if (now - prev < LOOP_INTERVAL_MS) {
        return;   // noch nicht dran
    }
    const float dt = (now - prev) / 1000.0f;   // normalerweise ~0.02 s
    prev = now;

    // 1. Bahn fortschreiben (SMOOTH-Bewegung)
    planner.update(dt);

    // 2. IK + Servos aktualisieren
    moveToPosition(planner.x(), planner.y());

    // 3. Eingabe/Modi: hier spaeter Joystick / BLE / Web-UI / RANDOM / RECORD ...
    //
    // Einfache Demo (bitte durch echte Modi ersetzen): alle 2 s weiterblaettern.
    static uint32_t demoLast = 0;
    static size_t   demoIdx  = 0;
    static const float demo[][2] = {
        {  0.0f, 170.0f },   // versteckt
        {-50.0f, 150.0f },   // Links-Peek
        {  0.0f, 170.0f },   // versteckt
        { 50.0f, 150.0f },   // Rechts-Peek
        {  0.0f, 190.0f },   // Oben-Peek
    };
    if (now - demoLast > 2000) {
        demoLast = now;
        demoIdx = (demoIdx + 1) % (sizeof(demo) / sizeof(demo[0]));
        planner.moveTo(demo[demoIdx][0], demo[demoIdx][1],
                       MotionPlanner::MoveType::SMOOTH);
    }
}
