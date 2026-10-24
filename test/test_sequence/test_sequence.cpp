// Unit-Tests fuer das Sequence-Datenmodell (JSON-Ablauf).

#include <unity.h>

#include <Sequence.h>

#include <string>

void setUp(void) {}
void tearDown(void) {}

static Sequence makeFullSequence() {
    Sequence s;
    s.name = "Test-Show";
    s.loop = true;

    Sequence::Step st;
    st.type = Sequence::StepType::PRESET;
    st.preset.name = "hide";
    st.preset.speed = 100.0f;
    st.preset.holdMs = 1500;
    s.steps.push_back(st);

    st = Sequence::Step{};
    st.type = Sequence::StepType::MOVE;
    st.move.x = 0.0f;
    st.move.y = 160.0f;
    st.move.speed = 35.0f;
    st.move.holdMs = 2500;
    s.steps.push_back(st);

    return s;
}

void test_default_empty(void) {
    Sequence s;
    TEST_ASSERT_EQUAL_UINT32(1, s.version);
    TEST_ASSERT_TRUE(s.loop);
    TEST_ASSERT_TRUE(s.steps.empty());
}

void test_roundtrip(void) {
    Sequence s = makeFullSequence();
    std::string json = Sequence::toJson(s);

    Sequence out;
    std::string err;
    TEST_ASSERT_TRUE(Sequence::fromJson(json.c_str(), out, &err));

    TEST_ASSERT_EQUAL_UINT32(1, out.version);
    TEST_ASSERT_EQUAL_STRING("Test-Show", out.name.c_str());
    TEST_ASSERT_TRUE(out.loop);
    TEST_ASSERT_EQUAL_UINT(2u, (unsigned)out.steps.size());

    TEST_ASSERT_TRUE(out.steps[0].type == Sequence::StepType::PRESET);
    TEST_ASSERT_EQUAL_STRING("hide", out.steps[0].preset.name.c_str());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, out.steps[0].preset.speed);
    TEST_ASSERT_EQUAL_UINT32(1500, out.steps[0].preset.holdMs);

    TEST_ASSERT_TRUE(out.steps[1].type == Sequence::StepType::MOVE);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.steps[1].move.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 160.0f, out.steps[1].move.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.0f, out.steps[1].move.speed);
    TEST_ASSERT_EQUAL_UINT32(2500, out.steps[1].move.holdMs);
}

void test_speed_clamp(void) {
    Sequence s;
    std::string err;
    const char* json = R"({"version":1,"name":"x","loop":true,"steps":[
        {"type":"move","x":0,"y":0,"speed":150},
        {"type":"move","x":0,"y":0,"speed":-20}
    ]})";
    TEST_ASSERT_TRUE(Sequence::fromJson(json, s, &err));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, s.steps[0].move.speed);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.steps[1].move.speed);
}

void test_unknown_step_type(void) {
    Sequence s;
    std::string err;
    const char* json = R"({"version":1,"steps":[{"type":"foo"}]})";
    TEST_ASSERT_FALSE(Sequence::fromJson(json, s, &err));
    TEST_ASSERT_TRUE(err.find("foo") != std::string::npos);
}

void test_unknown_preset(void) {
    Sequence s;
    std::string err;
    const char* json = R"({"version":1,"steps":[{"type":"preset","name":"top"}]})";
    TEST_ASSERT_FALSE(Sequence::fromJson(json, s, &err));
}

void test_move_out_of_bounds(void) {
    Sequence s;
    std::string err;
    const char* json = R"({"version":1,"steps":[{"type":"move","x":300,"y":0}]})";
    TEST_ASSERT_FALSE(Sequence::fromJson(json, s, &err));
}

void test_bad_json(void) {
    Sequence s;
    std::string err;
    TEST_ASSERT_FALSE(Sequence::fromJson("not json", s, &err));
}

void test_version_mismatch(void) {
    Sequence s;
    std::string err;
    const char* json = R"({"version":2,"steps":[]})";
    TEST_ASSERT_FALSE(Sequence::fromJson(json, s, &err));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_default_empty);
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_speed_clamp);
    RUN_TEST(test_unknown_step_type);
    RUN_TEST(test_unknown_preset);
    RUN_TEST(test_move_out_of_bounds);
    RUN_TEST(test_bad_json);
    RUN_TEST(test_version_mismatch);
    return UNITY_END();
}
