#pragma once

#include <cstdint>
#include <limits>

inline constexpr int STEERING_ENCODER_CENTER = 3220;
inline constexpr double STEERING_DEGREES_PER_ENCODER_INCREMENT = 0.00553967623263;

// Sample at 5 ms intervals: motion between valid readings must be < half a turn.
// Homing identifies which encoder revolution contains the calibrated center.
class SteeringAngle {
public:
    void reset() { referenced_ = false; }
    void reference(int raw, uint32_t now) {
        counts_ = raw - STEERING_ENCODER_CENTER;
        previous_ = raw;
        last_valid_ = now;
        referenced_ = valid_ = true;
    }
    void update(int raw, bool valid, uint32_t now) {
        valid_ = valid;
        if (!referenced_) return;
        // A long feedback gap makes the number of wraps ambiguous; rehome to recover.
        if (now - last_valid_ > 100) { reset(); return; }
        if (!valid) return;
        int delta = raw - previous_;
        if (delta > 2048) delta -= 4096;
        if (delta < -2048) delta += 4096;
        counts_ += delta;
        previous_ = raw;
        last_valid_ = now;
    }
    float degrees() const {
        // Match the host's existing convention: positive left, negative right.
        return referenced_ && valid_
            ? static_cast<float>(-counts_ * STEERING_DEGREES_PER_ENCODER_INCREMENT)
            : std::numeric_limits<float>::quiet_NaN();
    }
private:
    int32_t counts_ = 0;
    int previous_ = 0;
    uint32_t last_valid_ = 0;
    bool referenced_ = false, valid_ = false;
};
