// Sequence.cpp

#include "Sequence.h"

#include <ArduinoJson.h>
#include <cstring>

namespace {

constexpr float kSpeedMin = 0.0f;
constexpr float kSpeedMax = 100.0f;

// Arbeitsraum-Grenzen (passend zur Workspace-Karte, mm).
constexpr float kXMin = -220.0f;
constexpr float kXMax = 220.0f;
constexpr float kYMin = -150.0f;
constexpr float kYMax = 215.0f;

// Kapazitaet des JSON-Puffers: reicht fuer Ablaeufe mit deutlich > 200 Schritten.
constexpr size_t kJsonCapacity = 32768;

float clampSpeed(float speed) {
    if (speed < kSpeedMin) {
        return kSpeedMin;
    }
    if (speed > kSpeedMax) {
        return kSpeedMax;
    }
    return speed;
}

bool isValidPresetName(const std::string& name) {
    // "top" ist bewusst NICHT enthalten: es ist die Montage-/Kalibrier-Referenz
    // (beide Kurbeln senkrecht), kein normaler Ablauf-Punkt.
    return name == "left" || name == "right" || name == "hide";
}

void serializeToDoc(const Sequence& seq, JsonDocument& doc) {
    JsonObject root = doc.to<JsonObject>();
    root["version"] = seq.version;
    root["name"] = seq.name;
    root["loop"] = seq.loop;
    root["autostart"] = seq.autostart;

    JsonArray steps = root.createNestedArray("steps");
    for (const auto& step : seq.steps) {
        JsonObject o = steps.createNestedObject();
        switch (step.type) {
            case Sequence::StepType::MOVE:
                o["type"] = "move";
                o["x"] = step.move.x;
                o["y"] = step.move.y;
                o["speed"] = step.move.speed;
                o["holdMs"] = step.move.holdMs;
                break;

            case Sequence::StepType::PRESET:
                o["type"] = "preset";
                o["name"] = step.preset.name;
                o["speed"] = step.preset.speed;
                o["holdMs"] = step.preset.holdMs;
                break;
        }
    }
}

}  // namespace

bool Sequence::validate(Sequence& seq, std::string* error) {
    if (seq.version != 1) {
        if (error) {
            *error = "unsupported version";
        }
        return false;
    }

    for (auto& step : seq.steps) {
        switch (step.type) {
            case StepType::MOVE:
                if (step.move.x < kXMin || step.move.x > kXMax ||
                    step.move.y < kYMin || step.move.y > kYMax) {
                    if (error) {
                        *error = "move out of bounds";
                    }
                    return false;
                }
                step.move.speed = clampSpeed(step.move.speed);
                break;

            case StepType::PRESET:
                if (!isValidPresetName(step.preset.name)) {
                    if (error) {
                        *error = "unknown preset: " + step.preset.name;
                    }
                    return false;
                }
                step.preset.speed = clampSpeed(step.preset.speed);
                break;
        }
    }
    return true;
}

bool Sequence::fromJson(const char* json, Sequence& out, std::string* error) {
    out = Sequence{};
    if (json == nullptr || json[0] == '\0') {
        if (error) {
            *error = "empty json";
        }
        return false;
    }

    DynamicJsonDocument doc(kJsonCapacity);
    const DeserializationError err = deserializeJson(doc, json);
    if (err) {
        if (error) {
            *error = err.c_str();
        }
        return false;
    }

    const JsonObject root = doc.as<JsonObject>();
    if (root.isNull()) {
        if (error) {
            *error = "root must be an object";
        }
        return false;
    }

    out.version = root["version"] | 1u;
    out.name = root["name"] | "";
    out.loop = root["loop"] | true;
    out.autostart = root["autostart"] | false;

    const JsonArray steps = root["steps"].as<JsonArray>();
    if (steps.isNull()) {
        if (error) {
            *error = "missing steps";
        }
        return false;
    }

    for (JsonObject s : steps) {
        Step step;
        const char* type = s["type"] | "";
        if (std::strcmp(type, "move") == 0) {
            step.type = StepType::MOVE;
            step.move.x = s["x"] | 0.0f;
            step.move.y = s["y"] | 0.0f;
            step.move.speed = s["speed"] | 100.0f;
            step.move.holdMs = s["holdMs"] | 0u;
        } else if (std::strcmp(type, "preset") == 0) {
            step.type = StepType::PRESET;
            step.preset.name = s["name"] | "";
            step.preset.speed = s["speed"] | 100.0f;
            step.preset.holdMs = s["holdMs"] | 0u;
        } else {
            if (error) {
                *error = std::string("unknown step type: ") + type;
            }
            return false;
        }
        out.steps.push_back(step);
    }

    return validate(out, error);
}

std::string Sequence::toJson(const Sequence& seq) {
    DynamicJsonDocument doc(kJsonCapacity);
    serializeToDoc(seq, doc);

    std::string result;
    serializeJsonPretty(doc, result);
    return result;
}

std::string Sequence::toJsonCompact(const Sequence& seq) {
    DynamicJsonDocument doc(kJsonCapacity);
    serializeToDoc(seq, doc);

    std::string result;
    serializeJson(doc, result);
    return result;
}
