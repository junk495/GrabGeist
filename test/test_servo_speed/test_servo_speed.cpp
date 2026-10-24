// Unit-Tests fuer das Geschwindigkeits-/Strombegrenzungs-Modell.

#include <unity.h>

#include <ServoSpeed.h>

void setUp(void) {}
void tearDown(void) {}

void test_clamp_percent(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ServoSpeed::clampPercent(-10.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, ServoSpeed::clampPercent(150.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, ServoSpeed::clampPercent(50.0f));
}

void test_clamp_rate(void) {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ServoSpeed::clampRate(-5.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ServoSpeed::clampRate(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ServoSpeed::clampRate(1000.0f));
    TEST_ASSERT_EQUAL_FLOAT(ServoSpeed::SERVO_MAX_RATE_US_PER_S,
                            ServoSpeed::clampRate(99999.0f));
}

void test_rate_full(void) {
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ServoSpeed::rateForPercent(1000.0f, 100.0f));
}

void test_rate_half(void) {
    TEST_ASSERT_EQUAL_FLOAT(500.0f, ServoSpeed::rateForPercent(1000.0f, 50.0f));
}

void test_rate_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ServoSpeed::rateForPercent(1000.0f, 0.0f));
}

void test_rate_clamped_percent(void) {
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ServoSpeed::rateForPercent(1000.0f, 150.0f));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_clamp_percent);
    RUN_TEST(test_clamp_rate);
    RUN_TEST(test_rate_full);
    RUN_TEST(test_rate_half);
    RUN_TEST(test_rate_zero);
    RUN_TEST(test_rate_clamped_percent);
    return UNITY_END();
}
