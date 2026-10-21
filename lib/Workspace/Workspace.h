// Workspace.h
// Berechnet und verwaltet den erreichbaren Arbeitsraum (Nierenform) auf einem
// Raster und bietet Pfadplanung (A*) und "Snapping" auf den naechsten gueltigen Punkt.
// Reine Logik - hardwareunabhaengig und testbar.

#pragma once

#include <cstdint>
#include <vector>

#include <Point.h>

class Workspace {
public:
    // Raster: X/Y-Bereich und Schrittweite (mm).
    // step = 5 mm ist ein guter Kompromiss aus Aufloesung und ESP32-RAM.
    Workspace(float xMin = -220.0f, float xMax = 220.0f,
              float yMin = -150.0f, float yMax = 215.0f, float step = 5.0f);

    // Karte berechnen (einmalig beim Start; nutzt Kinematics::isReachable).
    void generate();

    // Abfrage (auf der Karte).
    bool isValid(float x, float y) const;

    // Naechster gueltiger Punkt (BFS; fuer das "Snappen" beim Tippen).
    bool nearestValid(float x, float y, float& outX, float& outY) const;

    // Pfad von Start nach Ziel (A*; Start/Ziel muessen gueltig sein).
    // Wegpunkte [mm] inkl. Start und Ziel. false, wenn kein Pfad existiert.
    bool findPath(float startX, float startY, float endX, float endY,
                  std::vector<Point>& waypoints) const;

    // Metadaten (fuer die UI-Uebertragung).
    int width() const { return width_; }
    int height() const { return height_; }
    float xMin() const { return xMin_; }
    float yMin() const { return yMin_; }
    float step() const { return step_; }
    const std::vector<uint8_t>& grid() const { return grid_; }

private:
    int toCellX(float x) const;
    int toCellY(float y) const;
    float toX(int cx) const;
    float toY(int cy) const;
    int index(int cx, int cy) const;
    bool inBounds(int cx, int cy) const;
    bool lineClear(int cx0, int cy0, int cx1, int cy1) const;
    float heuristic(int cx, int cy, int ex, int ey) const;
    bool aStar(int sCx, int sCy, int eCx, int eCy, std::vector<Point>& waypoints) const;

    float xMin_, xMax_, yMin_, yMax_, step_;
    int width_ = 0;
    int height_ = 0;
    std::vector<uint8_t> grid_;
};
