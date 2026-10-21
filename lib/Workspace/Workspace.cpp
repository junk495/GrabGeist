// Workspace.cpp

#include "Workspace.h"

#include "Kinematics.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>

Workspace::Workspace(float xMin, float xMax, float yMin, float yMax, float step)
    : xMin_(xMin), xMax_(xMax), yMin_(yMin), yMax_(yMax), step_(step) {
    width_ = static_cast<int>(std::round((xMax_ - xMin_) / step_)) + 1;
    height_ = static_cast<int>(std::round((yMax_ - yMin_) / step_)) + 1;
    grid_.assign(static_cast<size_t>(width_ * height_), 0);
}

void Workspace::generate() {
    for (int cy = 0; cy < height_; ++cy) {
        const float y = toY(cy);
        for (int cx = 0; cx < width_; ++cx) {
            const float x = toX(cx);
            grid_[index(cx, cy)] = Kinematics::isReachable(x, y) ? 1 : 0;
        }
    }
}

bool Workspace::isValid(float x, float y) const {
    const int cx = toCellX(x);
    const int cy = toCellY(y);
    return inBounds(cx, cy) && grid_[index(cx, cy)] != 0;
}

bool Workspace::nearestValid(float x, float y, float& outX, float& outY) const {
    int cx = toCellX(x);
    int cy = toCellY(y);
    // Punkt ausserhalb des Rasters auf den naechsten Rand clampen
    if (cx < 0) {
        cx = 0;
    } else if (cx >= width_) {
        cx = width_ - 1;
    }
    if (cy < 0) {
        cy = 0;
    } else if (cy >= height_) {
        cy = height_ - 1;
    }

    std::vector<uint8_t> visited(static_cast<size_t>(width_ * height_), 0);
    std::queue<int> q;
    q.push(index(cx, cy));
    visited[index(cx, cy)] = 1;

    while (!q.empty()) {
        const int idx = q.front();
        q.pop();
        const int curX = idx % width_;
        const int curY = idx / width_;
        if (grid_[idx]) {
            outX = toX(curX);
            outY = toY(curY);
            return true;
        }
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = curX + dx;
                const int ny = curY + dy;
                if (!inBounds(nx, ny)) {
                    continue;
                }
                const int nidx = index(nx, ny);
                if (!visited[nidx]) {
                    visited[nidx] = 1;
                    q.push(nidx);
                }
            }
        }
    }
    return false;
}

int Workspace::toCellX(float x) const {
    return static_cast<int>(std::round((x - xMin_) / step_));
}

int Workspace::toCellY(float y) const {
    return static_cast<int>(std::round((y - yMin_) / step_));
}

float Workspace::toX(int cx) const {
    return xMin_ + cx * step_;
}

float Workspace::toY(int cy) const {
    return yMin_ + cy * step_;
}

int Workspace::index(int cx, int cy) const {
    return cy * width_ + cx;
}

bool Workspace::inBounds(int cx, int cy) const {
    return cx >= 0 && cx < width_ && cy >= 0 && cy < height_;
}
bool Workspace::findPath(float startX, float startY, float endX, float endY,
                         std::vector<Point>& waypoints) const {
    waypoints.clear();
    const int sCx = toCellX(startX);
    const int sCy = toCellY(startY);
    const int eCx = toCellX(endX);
    const int eCy = toCellY(endY);

    if (!inBounds(sCx, sCy) || !inBounds(eCx, eCy)) {
        return false;
    }
    if (!grid_[index(sCx, sCy)] || !grid_[index(eCx, eCy)]) {
        return false;   // Start/Ziel muessen gueltig sein
    }

    // Schnellpfad: ist die gerade Linie frei?
    if (lineClear(sCx, sCy, eCx, eCy)) {
        waypoints.push_back({startX, startY});
        waypoints.push_back({endX, endY});
        return true;
    }

    return aStar(sCx, sCy, eCx, eCy, waypoints);
}

bool Workspace::lineClear(int cx0, int cy0, int cx1, int cy1) const {
    const int dx = cx1 > cx0 ? cx1 - cx0 : cx0 - cx1;
    const int dy = -(cy1 > cy0 ? cy1 - cy0 : cy0 - cy1);
    const int sx = cx0 < cx1 ? 1 : -1;
    const int sy = cy0 < cy1 ? 1 : -1;
    int err = dx + dy;
    int cx = cx0;
    int cy = cy0;

    while (true) {
        if (!grid_[index(cx, cy)]) {
            return false;
        }
        if (cx == cx1 && cy == cy1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            cx += sx;
        }
        if (e2 <= dx) {
            err += dx;
            cy += sy;
        }
    }
    return true;
}

float Workspace::heuristic(int cx, int cy, int ex, int ey) const {
    const float dx = static_cast<float>(cx - ex);
    const float dy = static_cast<float>(cy - ey);
    return std::sqrt(dx * dx + dy * dy);
}

bool Workspace::aStar(int sCx, int sCy, int eCx, int eCy, std::vector<Point>& waypoints) const {
    const int n = width_ * height_;
    const int startIdx = index(sCx, sCy);
    const int endIdx = index(eCx, eCy);

    std::vector<float> gScore(static_cast<size_t>(n), 1e30f);
    std::vector<int> cameFrom(static_cast<size_t>(n), -1);
    std::vector<uint8_t> closed(static_cast<size_t>(n), 0);

    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    gScore[startIdx] = 0.0f;
    open.push({heuristic(sCx, sCy, eCx, eCy), startIdx});

    while (!open.empty()) {
        const Node top = open.top();
        open.pop();
        const int idx = top.second;
        if (closed[idx]) {
            continue;
        }
        closed[idx] = 1;

        if (idx == endIdx) {
            // Pfad rekonstruieren
            std::vector<int> cells;
            int cur = endIdx;
            while (cur != -1) {
                cells.push_back(cur);
                if (cur == startIdx) {
                    break;
                }
                cur = cameFrom[cur];
            }
            std::reverse(cells.begin(), cells.end());
            waypoints.clear();
            for (int c : cells) {
                waypoints.push_back({toX(c % width_), toY(c / width_)});
            }
            return true;
        }

        const int cx = idx % width_;
        const int cy = idx / width_;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (!inBounds(nx, ny)) {
                    continue;
                }
                const int nidx = index(nx, ny);
                if (!grid_[nidx] || closed[nidx]) {
                    continue;
                }
                const float stepCost = (dx != 0 && dy != 0) ? 1.41421356f : 1.0f;
                const float ng = gScore[idx] + stepCost;
                if (ng < gScore[nidx]) {
                    gScore[nidx] = ng;
                    cameFrom[nidx] = idx;
                    open.push({ng + heuristic(nx, ny, eCx, eCy), nidx});
                }
            }
        }
    }
    return false;
}
