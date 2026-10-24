// Unit-Tests fuer den CommandParser (Text -> Befehl).

#include <unity.h>

#include <CommandParser.h>

void setUp(void) {}
void tearDown(void) {}

void test_move(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("MOVE 10 20", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::MOVE);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, c.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, c.y);
    TEST_ASSERT_FALSE(c.snap);
}

void test_move_snap(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("MOVE -50 150 snap", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::MOVE);
    TEST_ASSERT_TRUE(c.snap);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, c.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 150.0f, c.y);
}

void test_move_case_insensitive(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("move 0 170", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::MOVE);
}

void test_peek(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("PEEK left", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::PEEK);
    TEST_ASSERT_TRUE(c.peek == Command::Peek::LEFT);
}

void test_peek_all_targets(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("PEEK right", c));
    TEST_ASSERT_TRUE(c.peek == Command::Peek::RIGHT);
    TEST_ASSERT_TRUE(CommandParser::parse("PEEK top", c));
    TEST_ASSERT_TRUE(c.peek == Command::Peek::TOP);
    TEST_ASSERT_TRUE(CommandParser::parse("PEEK hide", c));
    TEST_ASSERT_TRUE(c.peek == Command::Peek::HIDE);
}

void test_stop_status(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("STOP", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::STOP);
    TEST_ASSERT_TRUE(CommandParser::parse("STATUS", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::STATUS);
}

void test_invalid(void) {
    Command c;
    TEST_ASSERT_FALSE(CommandParser::parse("", c));
    TEST_ASSERT_FALSE(CommandParser::parse(nullptr, c));
    TEST_ASSERT_FALSE(CommandParser::parse("FOO", c));
    TEST_ASSERT_FALSE(CommandParser::parse("MOVE 10", c));        // y fehlt
    TEST_ASSERT_FALSE(CommandParser::parse("PEEK", c));           // Ziel fehlt
    TEST_ASSERT_FALSE(CommandParser::parse("PEEK banana", c));    // unbekanntes Ziel
    TEST_ASSERT_FALSE(CommandParser::parse("RAW 1500", c));       // rechter Puls fehlt
}

void test_raw(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("RAW 500 2500", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::RAW);
    TEST_ASSERT_EQUAL_UINT16(500, c.rawL);
    TEST_ASSERT_EQUAL_UINT16(2500, c.rawR);
}

void test_raw_clamp(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("raw 100 3000", c));    // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::RAW);
    TEST_ASSERT_EQUAL_UINT16(500, c.rawL);    // 100 -> 500 geklemmt
    TEST_ASSERT_EQUAL_UINT16(2500, c.rawR);   // 3000 -> 2500 geklemmt
}

void test_auto(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("AUTO", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::AUTO);
    TEST_ASSERT_TRUE(CommandParser::parse("auto", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::AUTO);
}

void test_resetlog(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("RESETLOG", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::RESETLOG);
    TEST_ASSERT_TRUE(CommandParser::parse("resetlog", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::RESETLOG);
}

void test_ramp(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("RAMP 750", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::RAMP);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 750.0f, c.ramp);

    TEST_ASSERT_TRUE(CommandParser::parse("RAMP", c));   // Abfrage ohne Wert
    TEST_ASSERT_TRUE(c.type == Command::Type::RAMP);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, c.ramp);

    TEST_ASSERT_TRUE(CommandParser::parse("ramp 1000", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::RAMP);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, c.ramp);
}

void test_speed(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("SPEED 50", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::SPEED);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, c.speed);

    TEST_ASSERT_TRUE(CommandParser::parse("SPEED", c));   // Abfrage ohne Wert
    TEST_ASSERT_TRUE(c.type == Command::Type::SPEED);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, c.speed);

    TEST_ASSERT_TRUE(CommandParser::parse("speed 100", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::SPEED);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, c.speed);
}

void test_wifi(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("WIFI off", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::WIFI);
    TEST_ASSERT_EQUAL_STRING("off", c.wifiMode.c_str());

    TEST_ASSERT_TRUE(CommandParser::parse("WIFI", c));   // Abfrage ohne Wert
    TEST_ASSERT_TRUE(c.type == Command::Type::WIFI);
    TEST_ASSERT_EQUAL_STRING("", c.wifiMode.c_str());
}

void test_random(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("RANDOM", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::RANDOM);
    TEST_ASSERT_TRUE(CommandParser::parse("random", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::RANDOM);
}

void test_rec_play(void) {
    Command c;
    TEST_ASSERT_TRUE(CommandParser::parse("REC", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::REC);
    TEST_ASSERT_TRUE(CommandParser::parse("PLAY", c));
    TEST_ASSERT_TRUE(c.type == Command::Type::PLAY);
    TEST_ASSERT_TRUE(CommandParser::parse("play", c));   // klein geschrieben
    TEST_ASSERT_TRUE(c.type == Command::Type::PLAY);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_move);
    RUN_TEST(test_move_snap);
    RUN_TEST(test_move_case_insensitive);
    RUN_TEST(test_peek);
    RUN_TEST(test_peek_all_targets);
    RUN_TEST(test_stop_status);
    RUN_TEST(test_invalid);
    RUN_TEST(test_raw);
    RUN_TEST(test_raw_clamp);
    RUN_TEST(test_auto);
    RUN_TEST(test_resetlog);
    RUN_TEST(test_ramp);
    RUN_TEST(test_speed);
    RUN_TEST(test_wifi);
    RUN_TEST(test_random);
    RUN_TEST(test_rec_play);
    return UNITY_END();
}
