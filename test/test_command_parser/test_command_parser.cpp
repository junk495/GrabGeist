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
    return UNITY_END();
}
