// Sequence.h
// Ablauf-Datenmodell (Phase 1): eine benannte, schleifenfaehige Schrittliste.
// Rein logisch + JSON; hardwarefrei und nativ testbar (ArduinoJson nur in der .cpp).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Sequence {
    enum class StepType { MOVE, PRESET, DELAY, WAIT };

    struct MoveStep {
        float x = 0.0f;
        float y = 0.0f;
        float speed = 100.0f;   // 0..100 % der Strombegrenzung
    };

    struct PresetStep {
        std::string name;       // "left" | "right" | "hide"
        float speed = 100.0f;
    };

    struct DelayStep {
        uint32_t ms = 0;
    };

    struct Step {
        StepType type = StepType::MOVE;
        MoveStep move;
        PresetStep preset;
        DelayStep delay;
    };

    uint32_t version = 1;
    std::string name;
    bool loop = true;
    std::vector<Step> steps;

    // JSON -> Sequence. Liefert false bei ungueltigem JSON/Schema.
    static bool fromJson(const char* json, Sequence& out, std::string* error = nullptr);

    // Sequence -> JSON-String (lesbar, fuer Export/Datei).
    static std::string toJson(const Sequence& seq);

    // Normalisiert (speed auf [0,100] klemmen) und prueft das Schema.
    static bool validate(Sequence& seq, std::string* error = nullptr);
};
