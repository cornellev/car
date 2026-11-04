#pragma once

#include <cstdint>
#include "steering_angle.hpp"

// Updated every 5 ms with fresh AS5600 feedback. Positive pulses turn right.
class SteeringHoming {
public:
    enum class State { idle, right, drain_right, center, drain_center, ready, failed };
    State state = State::idle;
    int right_bound = 0;

    void start(uint32_t now) { state = State::right; started_ = now; initialized_ = false; in_wall_window_ = false; }
    void reset() { state = State::idle; }
    bool active() const { return state != State::idle && state != State::ready && state != State::failed; }

    // Returns requested jog direction; zero means stop and drain the PWM pipeline.
    int update(uint32_t now, int raw, bool valid, bool drained) {
        if (!active()) return 0;
        if (!valid || now - started_ > 15000) { state = State::failed; return 0; }
        if (!initialized_) { previous_ = raw; initialized_ = true; }
        int delta = raw - previous_;
        if (delta > 2048) delta -= 4096;
        if (delta < -2048) delta += 4096;
        previous_ = raw;
        if (state == State::right) {
            // Require continuous residence in the window, not a stationary reading.
            if (raw >= 1700 && raw <= 2100) {
                if (!in_wall_window_) {
                    in_wall_window_ = true;
                    wall_entered_ = now;
                }
                if (now - wall_entered_ >= 200) state = State::drain_right;
            } else {
                in_wall_window_ = false;
            }
        } else if (state == State::drain_right && drained) {
            right_bound = raw;
            remaining_ = (raw - STEERING_ENCODER_CENTER + 4096) % 4096;
            state = remaining_ <= 16 ? State::drain_center : State::center;
        } else if (state == State::center) {
            remaining_ += delta;  // Unwrap incremental feedback, including 0/4095 crossings.
            if (remaining_ <= 16) state = State::drain_center;
        } else if (state == State::drain_center && drained) {
            state = State::ready;
        }
        return state == State::right ? 1 : state == State::center ? -1 : 0;
    }
private:
    uint32_t started_ = 0, wall_entered_ = 0;
    int previous_ = 0, remaining_ = 0;
    bool initialized_ = false;
    bool in_wall_window_ = false;
};
