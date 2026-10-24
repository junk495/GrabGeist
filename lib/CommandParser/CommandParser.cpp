// CommandParser.cpp

#include "CommandParser.h"

#include <cstdlib>   // strtof
#include <cstring>   // strtok, strncpy

namespace {

// Vergleicht zwei Strings ohne Gross-/Kleinschreibung.
bool equalsIgnoreCase(const char* a, const char* b) {
    while (*a != '\0' && *b != '\0') {
        char ca = (*a >= 'A' && *a <= 'Z') ? (*a - 'A' + 'a') : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (*b - 'A' + 'a') : *b;
        if (ca != cb) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

}  // namespace

bool CommandParser::parse(const char* line, Command& out) {
    out = Command{};
    if (line == nullptr || *line == '\0') {
        return false;
    }

    // Kopie anlegen, da strtok den String veraendert
    char buf[64];
    std::strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* keyword = std::strtok(buf, " \t\r\n");
    if (keyword == nullptr) {
        return false;
    }

    if (equalsIgnoreCase(keyword, "MOVE")) {
        char* xs = std::strtok(nullptr, " \t\r\n");
        char* ys = std::strtok(nullptr, " \t\r\n");
        if (xs == nullptr || ys == nullptr) {
            return false;   // braucht x und y
        }
        out.type = Command::Type::MOVE;
        out.x = std::strtof(xs, nullptr);
        out.y = std::strtof(ys, nullptr);
        char* opt = std::strtok(nullptr, " \t\r\n");
        out.snap = (opt != nullptr && equalsIgnoreCase(opt, "snap"));
        return true;
    }

    if (equalsIgnoreCase(keyword, "PEEK")) {
        char* target = std::strtok(nullptr, " \t\r\n");
        if (target == nullptr) {
            return false;
        }
        out.type = Command::Type::PEEK;
        if (equalsIgnoreCase(target, "left")) {
            out.peek = Command::Peek::LEFT;
        } else if (equalsIgnoreCase(target, "right")) {
            out.peek = Command::Peek::RIGHT;
        } else if (equalsIgnoreCase(target, "top")) {
            out.peek = Command::Peek::TOP;
        } else if (equalsIgnoreCase(target, "hide")) {
            out.peek = Command::Peek::HIDE;
        } else {
            return false;
        }
        return true;
    }

    if (equalsIgnoreCase(keyword, "STOP")) {
        out.type = Command::Type::STOP;
        return true;
    }

    if (equalsIgnoreCase(keyword, "STATUS")) {
        out.type = Command::Type::STATUS;
        return true;
    }

    if (equalsIgnoreCase(keyword, "RAW")) {
        char* ls = std::strtok(nullptr, " \t\r\n");
        char* rs = std::strtok(nullptr, " \t\r\n");
        if (ls == nullptr || rs == nullptr) {
            return false;   // braucht linken und rechten Puls
        }
        long l = std::strtol(ls, nullptr, 10);
        long r = std::strtol(rs, nullptr, 10);
        // Auf den Servo-Bereich klemmen
        if (l < 500)  l = 500;
        if (l > 2500) l = 2500;
        if (r < 500)  r = 500;
        if (r > 2500) r = 2500;
        out.type = Command::Type::RAW;
        out.rawL = static_cast<uint16_t>(l);
        out.rawR = static_cast<uint16_t>(r);
        return true;
    }

    if (equalsIgnoreCase(keyword, "AUTO")) {
        out.type = Command::Type::AUTO;
        return true;
    }

    if (equalsIgnoreCase(keyword, "MAP")) {
        out.type = Command::Type::MAP;
        return true;
    }

    if (equalsIgnoreCase(keyword, "RESETLOG")) {
        out.type = Command::Type::RESETLOG;
        return true;
    }

    if (equalsIgnoreCase(keyword, "RAMP")) {
        out.type = Command::Type::RAMP;
        char* vs = std::strtok(nullptr, " \t\r\n");
        if (vs != nullptr) {
            out.ramp = std::strtof(vs, nullptr);
        }
        return true;
    }

    if (equalsIgnoreCase(keyword, "SPEED")) {
        out.type = Command::Type::SPEED;
        char* vs = std::strtok(nullptr, " \t\r\n");
        if (vs != nullptr) {
            out.speed = std::strtof(vs, nullptr);
        }
        return true;
    }

    if (equalsIgnoreCase(keyword, "RANDOM")) {
        out.type = Command::Type::RANDOM;
        return true;
    }

    if (equalsIgnoreCase(keyword, "REC")) {
        out.type = Command::Type::REC;
        return true;
    }

    if (equalsIgnoreCase(keyword, "PLAY")) {
        out.type = Command::Type::PLAY;
        return true;
    }

    return false;   // unbekanntes Kommando
}
