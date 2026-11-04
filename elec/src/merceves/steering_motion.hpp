#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Open-loop position: zero is the physical position at boot. One unit is one pulse.
class SteeringMotion {
public:
    struct Step {
        int direction = 0;
        uint32_t interval_us = 0;
        float end_speed = 0.0f;
        float end_speed_squared = 0.0f;
    };

    SteeringMotion(int32_t half_range, float max_speed, float acceleration)
        : half_range_(half_range), max_speed_(max_speed), acceleration_(acceleration) {}

    void set_target(float normalized_position) {
        target_ = static_cast<int32_t>(std::lround(
            std::clamp(normalized_position, -1.0f, 1.0f) * half_range_));
    }

    Step next_step() const {
        const int32_t error = target_ - position_;
        if (speed_squared_ == 0.0f && error == 0) {
            return {};
        }
        const int direction = speed_squared_ != 0.0f ? direction_ : (error > 0 ? 1 : -1);
        const float speed = std::sqrt(speed_squared_);
        const int32_t distance = error * direction;
        // Brake when the target moves behind us; reverse only after reaching zero speed.
        float end_squared = distance > 0
            ? std::min(max_speed_ * max_speed_, speed_squared_ + 2.0f * acceleration_)
            : std::max(0.0f, speed_squared_ - 2.0f * acceleration_);
        if (distance > 0) {
            if (speed_squared_ <= 2.0f * acceleration_ * distance + 1.0f) {
                end_squared = std::min(end_squared, 2.0f * acceleration_ * (distance - 1));
            } else {
                // A closer target may be inside our stopping distance. Brake smoothly,
                // pass it if necessary, then return after coming to rest.
                end_squared = std::max(0.0f, speed_squared_ - 2.0f * acceleration_);
            }
        }
        // Even a retarget while braking must leave enough distance to stop at the travel limit.
        const int32_t limit_distance = half_range_ - position_ * direction;
        if (limit_distance <= 0) {
            return {};
        }
        end_squared = std::min(end_squared, 2.0f * acceleration_ * (limit_distance - 1));
        const float end_speed = std::sqrt(end_squared);

        // Boundary speeds determine the time for one step under constant acceleration.
        // A one-step move from rest uses a triangular profile within that step.
        const float seconds = speed + end_speed > 0.0f
            ? 2.0f / (speed + end_speed)
            : 1.0f / std::min(max_speed_, std::sqrt(acceleration_))
                + std::min(max_speed_, std::sqrt(acceleration_)) / acceleration_;
        return {direction, static_cast<uint32_t>(std::ceil(seconds * 1'000'000.0f)), end_speed, end_squared};
    }

    void step_taken(const Step& step) {
        position_ += step.direction;
        direction_ = step.direction;
        speed_squared_ = step.end_speed_squared;
    }

    void stop() {
        speed_squared_ = 0.0f;
        target_ = position_;
    }

    void zero() { position_ = 0; target_ = 0; speed_squared_ = 0.0f; }

    int32_t position() const { return position_; }

private:
    int32_t half_range_;
    float max_speed_;
    float acceleration_;
    int32_t position_ = 0;
    int32_t target_ = 0;
    // Store squared speed to avoid accumulating sqrt rounding errors across steps.
    float speed_squared_ = 0.0f;
    int direction_ = 1;
};
