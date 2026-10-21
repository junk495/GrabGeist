// Unit-Tests fuer die Workspace-Klasse (Karte, Snapping, Pfadplanung).

#include <unity.h>

#include <Workspace.h>

// Grobes Raster (10 mm) fuer schnelle Tests.
static Workspace makeWorkspace() {
    Workspace ws(-220.0f, 220.0f, -150.0f, 215.0f, 10.0f);
    ws.generate();
    return ws;
}

void setUp(void) {}
void tearDown(void) {}

void test_generate_query(void) {
    Workspace ws = makeWorkspace();
    TEST_ASSERT_TRUE(ws.isValid(0.0f, 170.0f));      // obere Kalotte
    TEST_ASSERT_FALSE(ws.isValid(0.0f, -40.0f));     // zentrale Sperrzone
    TEST_ASSERT_TRUE(ws.isValid(-120.0f, -40.0f));   // linke Tasche
    TEST_ASSERT_TRUE(ws.isValid(120.0f, -40.0f));    // rechte Tasche
}

void test_nearest_valid(void) {
    Workspace ws = makeWorkspace();
    float x = 0.0f, y = 0.0f;
    TEST_ASSERT_TRUE(ws.nearestValid(0.0f, -40.0f, x, y));
    TEST_ASSERT_TRUE(ws.isValid(x, y));              // Ergebnis muss gueltig sein
    // Sollte in der Naehe liegen (oberhalb der Sperrzone).
    const float dx = x - 0.0f;
    const float dy = y - (-40.0f);
    TEST_ASSERT_TRUE(dx * dx + dy * dy < 100.0f * 100.0f);
}

void test_findpath_direct(void) {
    Workspace ws = makeWorkspace();
    std::vector<Point> waypoints;
    TEST_ASSERT_TRUE(ws.findPath(0.0f, 170.0f, 50.0f, 150.0f, waypoints));
    TEST_ASSERT_EQUAL(2, waypoints.size());          // gerade Linie frei -> nur 2 Punkte
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, waypoints[0].x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 170.0f, waypoints[0].y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, waypoints[1].x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 150.0f, waypoints[1].y);
}

void test_findpath_around_center(void) {
    Workspace ws = makeWorkspace();
    std::vector<Point> waypoints;
    // Querbewegung: die gerade Linie kreuzt die Sperrzone -> A* muss herumfuehren.
    TEST_ASSERT_TRUE(ws.findPath(-120.0f, -40.0f, 120.0f, -40.0f, waypoints));
    TEST_ASSERT_TRUE(waypoints.size() > 2);          // Umweg noetig
    // Alle Wegpunkte muessen gueltig sein.
    for (const Point& p : waypoints) {
        TEST_ASSERT_TRUE(ws.isValid(p.x, p.y));
    }
    // Start und Ziel stimmen.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -120.0f, waypoints.front().x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -40.0f, waypoints.front().y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 120.0f, waypoints.back().x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -40.0f, waypoints.back().y);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_generate_query);
    RUN_TEST(test_nearest_valid);
    RUN_TEST(test_findpath_direct);
    RUN_TEST(test_findpath_around_center);
    return UNITY_END();
}
