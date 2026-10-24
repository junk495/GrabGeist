// Unit-Tests fuer die Kinematics-Klasse (Inverse Kinematik).

#include <unity.h>

#include <Kinematics.h>

void setUp(void) {}
void tearDown(void) {}

// Zentrum P(0, 170): thetaL ~ 123.1 Grad, thetaR ~ 56.9 Grad (L2 = 125 mm)
void test_ik_center(void) {
    float thetaL = 0.0f, thetaR = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(0.0f, 170.0f, thetaL, thetaR));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.1487f, thetaL);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9929f, thetaR);
}

// Zu weit ausserhalb der Reichweite (d > L1 + L2)
void test_ik_unreachable_far(void) {
    float thetaL = 0.0f, thetaR = 0.0f;
    TEST_ASSERT_FALSE(Kinematics::calculateIK(0.0f, 500.0f, thetaL, thetaR));
}

// Zu nah am Servo (d < |L1 - L2| + Sicherheitsabstand)
void test_ik_unreachable_near(void) {
    float thetaL = 0.0f, thetaR = 0.0f;
    TEST_ASSERT_FALSE(Kinematics::calculateIK(0.0f, 5.0f, thetaL, thetaR));
}

// Rundreise FK -> IK: die "Knie nach aussen"-Winkel muessen reproduziert werden.
void test_ik_roundtrip(void) {
    const float pairs[][2] = {
        {Kinematics::degToRad(120.0f), Kinematics::degToRad(60.0f)},
        {Kinematics::degToRad(110.0f), Kinematics::degToRad(70.0f)},
        {Kinematics::degToRad(100.0f), Kinematics::degToRad(80.0f)},
    };

    for (const auto& p : pairs) {
        float x = 0.0f, y = 0.0f, thetaL = 0.0f, thetaR = 0.0f;
        TEST_ASSERT_TRUE(Kinematics::forwardKinematics(p[0], p[1], x, y));
        TEST_ASSERT_TRUE(Kinematics::calculateIK(x, y, thetaL, thetaR));
        TEST_ASSERT_FLOAT_WITHIN(0.01f, p[0], thetaL);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, p[1], thetaR);
    }
}

// Bei X = 0 zeigt die Geist-Zentrallinie senkrecht nach oben (psi = 90 Grad).
void test_target_centerline_up(void) {
    float psi = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateTargetCenterlineAngle(0.0f, 170.0f, psi));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, Kinematics::degToRad(90.0f), psi);
}

// Untere "Nieren-Taschen" (Over-Center) sind erreichbar.
void test_lower_pocket_left(void) {
    TEST_ASSERT_TRUE(Kinematics::isReachable(-120.0f, -40.0f));
    float thetaL = 0.0f, thetaR = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(-120.0f, -40.0f, thetaL, thetaR));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(288.8f), thetaL);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(142.3f), thetaR);
}

void test_lower_pocket_right(void) {
    TEST_ASSERT_TRUE(Kinematics::isReachable(120.0f, -40.0f));
    float thetaL = 0.0f, thetaR = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(120.0f, -40.0f, thetaL, thetaR));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(37.7f), thetaL);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(-108.8f), thetaR);
}

// Zentrale Sperrzone (Servogehaeuse) ist blockiert.
void test_servo_box_blocked(void) {
    TEST_ASSERT_FALSE(Kinematics::isReachable(0.0f, -40.0f));
    TEST_ASSERT_FALSE(Kinematics::isReachable(0.0f, 0.0f));
    TEST_ASSERT_FALSE(Kinematics::isReachable(0.0f, 10.0f));
}

// Hysterese: der Hint waehlt die naechste gueltige symmetrische Konfiguration.
// (0,170) hat mehrere gueltige Loesungen; gemischte (ein Arm Over-Center) werden
// vermieden, damit der Geist nicht schief steht.
void test_hysteresis(void) {
    // Ohne Hint: Knie nach aussen.
    float tl = 0.0f, tr = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(0.0f, 170.0f, tl, tr));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(123.1f), tl);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(56.9f), tr);

    // Auch mit einem Hint nahe einer Over-Center-/Misch-Konfiguration bleibt
    // die symmetrische Loesung (beide "Knie nach aussen") gewaehlt.
    float tl2 = 0.0f, tr2 = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(0.0f, 170.0f,
                                             Kinematics::degToRad(35.0f),
                                             Kinematics::degToRad(145.0f), tl2, tr2));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(123.1f), tl2);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(56.9f), tr2);
}

// Untere Tasche: auch mit einem "falschen" Hint bleibt die (einzige gueltige)
// gemischte Over-Center-Loesung erhalten.
void test_lower_pocket_hysteresis(void) {
    float tl = 0.0f, tr = 0.0f;
    TEST_ASSERT_TRUE(Kinematics::calculateIK(-120.0f, -40.0f,
                                             Kinematics::degToRad(0.0f),
                                             Kinematics::degToRad(0.0f), tl, tr));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(288.8f), tl);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, Kinematics::degToRad(142.3f), tr);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ik_center);
    RUN_TEST(test_ik_unreachable_far);
    RUN_TEST(test_ik_unreachable_near);
    RUN_TEST(test_ik_roundtrip);
    RUN_TEST(test_target_centerline_up);
    RUN_TEST(test_lower_pocket_left);
    RUN_TEST(test_lower_pocket_right);
    RUN_TEST(test_servo_box_blocked);
    RUN_TEST(test_hysteresis);
    RUN_TEST(test_lower_pocket_hysteresis);
    return UNITY_END();
}
