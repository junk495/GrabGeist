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
    moving_ = false;
}

void MotionPlanner::moveTo(float x, float y, MoveType type) {
    if (type == MoveType::SNAP) {
        // Direkter Sprung: Servo bewegt sich mit maximaler Geschwindigkeit
        currentX_ = x;
        currentY_ = y;
        moving_ = false;
        return;
    }

    // SMOOTH: neue Bewegung ab der aktuellen (interpolierten) Position
    startX_ = currentX_;
    startY_ = currentY_;
    targetX_ = x;
    targetY_ = y;
    elapsed_ = 0.0f;
    moving_ = true;
}

bool MotionPlanner::update(float dt) {
    if (!moving_) {
        return false;
    }

    elapsed_ += dt;

    // Ziel erreicht
    if (elapsed_ >= duration_) {
        currentX_ = targetX_;
        currentY_ = targetY_;
        moving_ = false;
        return false;
    }

    // Fortschritt 0..1 und Easing anwenden
    const float p = elapsed_ / duration_;
    const float e = ease(p);
    currentX_ = startX_ + (targetX_ - startX_) * e;
    currentY_ = startY_ + (targetY_ - startY_) * e;
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
