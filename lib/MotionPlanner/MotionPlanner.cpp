// MotionPlanner.cpp

#include "MotionPlanner.h"

#include <cmath>

void MotionPlanner::setDuration(float seconds) {
    // Positive Dauer erzwingen (Schutz vor Division durch 0)
    duration_ = (seconds > 0.0f) ? seconds : 1.0f;
}

void MotionPlanner::setEasing(Easing easing) {
    easing_ = easing;
}

void MotionPlanner::setPosition(float x, float y) {
    currentX_ = x;
    currentY_ = y;
    path_.clear();
    moving_ = false;
}

void MotionPlanner::moveTo(float x, float y, MoveType type) {
    // Einzelziel = Weg mit zwei Punkten (Start = aktuelle Position).
    std::vector<Point> wp;
    wp.push_back({currentX_, currentY_});
    wp.push_back({x, y});
    moveAlongPath(wp, type);
}

void MotionPlanner::moveAlongPath(const std::vector<Point>& waypoints, MoveType type) {
    if (waypoints.empty()) {
        return;
    }
    if (type == MoveType::SNAP || waypoints.size() == 1) {
        // Direkter Sprung zum letzten Punkt
        currentX_ = waypoints.back().x;
        currentY_ = waypoints.back().y;
        path_.clear();
        moving_ = false;
        return;
    }

    // SMOOTH: Wegpunkte uebernehmen, Start = erster Punkt
    path_ = waypoints;
    currentX_ = path_[0].x;
    currentY_ = path_[0].y;
    seg_ = 0;
    elapsed_ = 0.0f;
    moving_ = true;
}

bool MotionPlanner::update(float dt) {
    if (!moving_) {
        return false;
    }

    elapsed_ += dt;
    const Point& a = path_[seg_];
    const Point& b = path_[seg_ + 1];

    // Segment abgeschlossen?
    if (elapsed_ >= duration_) {
        currentX_ = b.x;
        currentY_ = b.y;
        ++seg_;
        if (seg_ >= path_.size() - 1) {
            moving_ = false;   // Pfad fertig
            return false;
        }
        elapsed_ = 0.0f;       // naechstes Segment beginnt
        return true;
    }

    // Fortschritt 0..1 und Easing anwenden
    const float p = elapsed_ / duration_;
    const float e = ease(p);
    currentX_ = a.x + (b.x - a.x) * e;
    currentY_ = a.y + (b.y - a.y) * e;
    return true;
}

float MotionPlanner::x() const { return currentX_; }
float MotionPlanner::y() const { return currentY_; }
bool  MotionPlanner::isMoving() const { return moving_; }

float MotionPlanner::ease(float t) const {
    if (easing_ == Easing::LINEAR) {
        return t;
    }
    // COSINE (S-Kurve): weiches Anfahren und Abbremsen
    return (1.0f - std::cos(kPi * t)) / 2.0f;
}
