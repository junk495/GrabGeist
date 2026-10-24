// Unit-Tests fuer die ServoController-Klasse (Kurbelwinkel -> PWM-Pulsbreite).

#include <unity.h>

#include <Kinematics.h>
#include <ServoController.h>

void setUp(void) {}
void tearDown(void) {}

// Ohne Kalibrierung: 0 rad -> Mitte (1500 us)
void test_pulse_center(void) {
    ServoController sc;
    TEST_ASSERT_EQUAL_UINT16(1500, sc.angleToPulseUsLeft(0.0f));
}

// +135 Grad (halber Stellbereich) -> 2500 us
void test_pulse_max(void) {
    ServoController sc;
    TEST_ASSERT_UINT16_WITHIN(1, 2500, sc.angleToPulseUsLeft(Kinematics::degToRad(135.0f)));
}

// -135 Grad -> 500 us
void test_pulse_min(void) {
    ServoController sc;
    TEST_ASSERT_UINT16_WITHIN(1, 500, sc.angleToPulseUsLeft(Kinematics::degToRad(-135.0f)));
}

// Invertierung dreht die Richtung um
void test_pulse_invert(void) {
    ServoController sc;
    sc.setCalibrationLeft(0.0f, true);
    TEST_ASSERT_UINT16_WITHIN(1, 500,  sc.angleToPulseUsLeft(Kinematics::degToRad(135.0f)));
    TEST_ASSERT_UINT16_WITHIN(1, 2500, sc.angleToPulseUsLeft(Kinematics::degToRad(-135.0f)));
}

// Nullpunkt-Offset verschiebt die Mitte
void test_pulse_zero_offset(void) {
    ServoController sc;
    sc.setCalibrationLeft(Kinematics::degToRad(30.0f), false);
    TEST_ASSERT_UINT16_WITHIN(1, 1500, sc.angleToPulseUsLeft(Kinematics::degToRad(30.0f)));   // relativ 0
    TEST_ASSERT_UINT16_WITHIN(1, 2500, sc.angleToPulseUsLeft(Kinematics::degToRad(165.0f)));  // relativ +135 Grad
}

// Ausserhalb des Bereichs -> auf 500 / 2500 us klemmen
void test_pulse_clamp(void) {
    ServoController sc;
    TEST_ASSERT_EQUAL_UINT16(2500, sc.angleToPulseUsLeft(Kinematics::degToRad(300.0f)));
    TEST_ASSERT_EQUAL_UINT16(500,  sc.angleToPulseUsLeft(Kinematics::degToRad(-300.0f)));
}

// Inverse Abbildung: Pulsbreite -> Winkel ist konsistent zur Vorwaerts-Abbildung.
void test_pulse_angle_roundtrip(void) {
    ServoController sc;
    sc.setCalibrationLeft(Kinematics::degToRad(162.90f), false);
    sc.setCalibrationRight(Kinematics::degToRad(15.75f), false);

    // HIDE-Position (1580 / 1410 us) zurueck auf Winkel abbilden.
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(173.70f), sc.pulseToAngleUsLeft(1580.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(3.60f), sc.pulseToAngleUsRight(1410.0f));

    // Roundtrip: Winkel -> Puls -> Winkel.
    const float tl = Kinematics::degToRad(120.0f);
    const float tr = Kinematics::degToRad(60.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, tl, sc.pulseToAngleUsLeft(sc.angleToPulseUsLeft(tl)));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, tr, sc.pulseToAngleUsRight(sc.angleToPulseUsRight(tr)));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pulse_center);
    RUN_TEST(test_pulse_max);
    RUN_TEST(test_pulse_min);
    RUN_TEST(test_pulse_invert);
    RUN_TEST(test_pulse_zero_offset);
    RUN_TEST(test_pulse_clamp);
    RUN_TEST(test_pulse_angle_roundtrip);
    return UNITY_END();
}
