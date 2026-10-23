// Unit-Tests fuer die MotionPlanner-Klasse (Bahnplanung/Interpolation).

#include <unity.h>

#include <MotionPlanner.h>

void setUp(void) {}
void tearDown(void) {}

// Snap: sofort am Ziel, keine Bewegung
void test_snap(void) {
    MotionPlanner mp;
    mp.moveTo(100.0f, 150.0f, MotionPlanner::MoveType::SNAP);
    TEST_ASSERT_FALSE(mp.isMoving());
    TEST_ASSERT_EQUAL_FLOAT(100.0f, mp.x());
    TEST_ASSERT_EQUAL_FLOAT(150.0f, mp.y());
}

// Linear: konstante Geschwindigkeit
void test_smooth_linear(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setDuration(1.0f);
    mp.moveTo(100.0f, 0.0f, MotionPlanner::MoveType::SMOOTH);
    TEST_ASSERT_TRUE(mp.isMoving());

    TEST_ASSERT_TRUE(mp.update(0.25f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, mp.x());

    TEST_ASSERT_TRUE(mp.update(0.25f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());

    TEST_ASSERT_TRUE(mp.update(0.25f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 75.0f, mp.x());

    TEST_ASSERT_FALSE(mp.update(0.25f));  // Ziel erreicht -> fertig
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, mp.x());
    TEST_ASSERT_FALSE(mp.isMoving());
}

// Cosinus (S-Kurve): weiches Anfahren/Abbremsen
void test_smooth_cosine(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::COSINE);
    mp.setDuration(1.0f);
    mp.moveTo(100.0f, 0.0f, MotionPlanner::MoveType::SMOOTH);

    mp.update(0.25f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.6447f, mp.x());   // (1-cos(pi/4))/2

    mp.update(0.25f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());        // (1-cos(pi/2))/2 = 0.5

    mp.update(0.25f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 85.3553f, mp.x());     // (1-cos(3pi/4))/2

    mp.update(0.25f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, mp.x());
    TEST_ASSERT_FALSE(mp.isMoving());
}

// 2D-Bewegung: beide Achsen werden interpoliert
void test_smooth_2d(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setDuration(1.0f);
    mp.moveTo(40.0f, 80.0f, MotionPlanner::MoveType::SMOOTH);

    mp.update(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, mp.y());
}

// Dauer aendert die Geschwindigkeit
void test_duration(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setDuration(2.0f);
    mp.moveTo(100.0f, 0.0f, MotionPlanner::MoveType::SMOOTH);

    mp.update(1.0f);   // halbe Zeit
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());
    TEST_ASSERT_TRUE(mp.isMoving());

    mp.update(1.0f);   // fertig
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, mp.x());
    TEST_ASSERT_FALSE(mp.isMoving());
}

// Neustart waehrend einer Bewegung startet ab der aktuellen Position
void test_restart_mid_move(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setDuration(1.0f);
    mp.moveTo(100.0f, 0.0f, MotionPlanner::MoveType::SMOOTH);
    mp.update(0.5f);                                        // bei x=50
    mp.moveTo(100.0f, 0.0f, MotionPlanner::MoveType::SMOOTH); // Neustart ab x=50
    mp.update(0.5f);                                        // 50 + 50*0.5 = 75
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 75.0f, mp.x());
}

// Update im Leerlauf macht nichts
void test_idle_update(void) {
    MotionPlanner mp;
    TEST_ASSERT_FALSE(mp.update(0.1f));  // nicht am Bewegen
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mp.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mp.y());
}

// Wegpunkt-Folge (A*-Pfad): konstante Bahngeschwindigkeit (setSpeed), Segment fuer Segment.
void test_move_along_path_smooth(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setSpeed(100.0f);   // 100 mm/s
    std::vector<Point> path = {{0.0f, 0.0f}, {50.0f, 0.0f}, {50.0f, 50.0f}};
    mp.moveAlongPath(path, MotionPlanner::MoveType::SMOOTH);
    TEST_ASSERT_TRUE(mp.isMoving());

    // Segment 1: 50 mm @ 100 mm/s = 0,5 s
    mp.update(0.25f);   // halbes Segment 1
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, mp.y());

    mp.update(0.25f);   // Segment 1 fertig
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, mp.y());
    TEST_ASSERT_TRUE(mp.isMoving());

    mp.update(0.25f);   // halbes Segment 2
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, mp.y());

    mp.update(0.25f);   // fertig
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mp.y());
    TEST_ASSERT_FALSE(mp.isMoving());
}

// Geschwindigkeit bestimmt die Segmentdauer (laengere Strecke = laengere Dauer).
void test_speed(void) {
    MotionPlanner mp;
    mp.setEasing(MotionPlanner::Easing::LINEAR);
    mp.setSpeed(200.0f);   // 200 mm/s
    std::vector<Point> path = {{0.0f, 0.0f}, {200.0f, 0.0f}};
    mp.moveAlongPath(path, MotionPlanner::MoveType::SMOOTH);

    mp.update(0.5f);   // 100 mm bei 200 mm/s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, mp.x());
    TEST_ASSERT_TRUE(mp.isMoving());

    mp.update(0.5f);   // fertig
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, mp.x());
    TEST_ASSERT_FALSE(mp.isMoving());
}

// Wegpunkt-Folge mit SNAP springt zum letzten Punkt.
void test_move_along_path_snap(void) {
    MotionPlanner mp;
    std::vector<Point> path = {{0.0f, 0.0f}, {10.0f, 10.0f}, {20.0f, 20.0f}};
    mp.moveAlongPath(path, MotionPlanner::MoveType::SNAP);
    TEST_ASSERT_FALSE(mp.isMoving());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, mp.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, mp.y());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_snap);
    RUN_TEST(test_smooth_linear);
    RUN_TEST(test_smooth_cosine);
    RUN_TEST(test_smooth_2d);
    RUN_TEST(test_duration);
    RUN_TEST(test_restart_mid_move);
    RUN_TEST(test_idle_update);
    RUN_TEST(test_move_along_path_smooth);
    RUN_TEST(test_move_along_path_snap);
    RUN_TEST(test_speed);
    return UNITY_END();
}
