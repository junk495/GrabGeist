// CommandParser.h
// Zerlegt Textbefehle (eine Zeile) in einen strukturierten Befehl.
// Reine Logik - hardwareunabhaengig und testbar.

#pragma once

#include <cstdint>

// Geparster Befehl
struct Command {
    enum class Type { NONE, MOVE, PEEK, STOP, STATUS, RAW };
    enum class Peek { LEFT, RIGHT, TOP, HIDE };

    Type type = Type::NONE;
    float x = 0.0f;            // MOVE: Ziel X [mm]
    float y = 0.0f;            // MOVE: Ziel Y [mm]
    bool  snap = false;        // MOVE: true = Sprung, false = weiche Bewegung
    Peek  peek = Peek::HIDE;   // PEEK: Ziel-Preset
    uint16_t rawL = 1500;      // RAW: linker Servo-Puls [us]
    uint16_t rawR = 1500;      // RAW: rechter Servo-Puls [us]
};

class CommandParser {
public:
    // Zerlegt eine Zeile (ohne Zeilenumbruch) in einen Befehl.
    // Liefert false, wenn die Zeile nicht verstanden wurde.
    static bool parse(const char* line, Command& out);
};
