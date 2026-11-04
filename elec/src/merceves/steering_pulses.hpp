#pragma once

#include <algorithm>
#include <cstdint>

#include "steering_motion.hpp"

// Turns SteeringMotion steps into PWM periods, one step per period, at 1 PWM count per us.
//
// The pulse sits at the end of each period (output inverted, so it's high once the counter
// reaches level), which makes the wrap interrupt fire right as a pulse finishes. The PWM's
// TOP and level are double-buffered: values written during a period take effect at the next
// wrap. So at each wrap, the period that just started was planned on the previous wrap, and
// on_wrap() plans the one after it. Steps are committed to the motion planner when planned,
// up to two ahead of the pulses actually sent; position() counts only sent pulses.
class SteeringPulses {
public:
    struct Period {
        uint16_t top = 0;    // period length minus one, in us
        uint16_t level = 1;  // output is high for counts >= level, so top + 1 means no pulse
        int direction = 0;   // direction of this period's pulse, 0 for none
    };

    struct Wrap {
        Period next;            // write into the buffered TOP and level
        int direction_now = 0;  // set the dir pin to this now, for the period that just started; 0 leaves it
    };

    // pulse_us: how long each pulse is high (the low part is at least as long).
    // idle_us: period length while there's nothing to step, i.e. how often to re-plan.
    // max_us: longest period; must be below 65535 so level = top + 1 still fits.
    SteeringPulses(SteeringMotion& motion, uint16_t pulse_us, uint16_t idle_us, uint16_t max_us)
        : motion_(motion), pulse_us_(pulse_us), idle_us_(idle_us), max_us_(max_us) {}

    // call at every wrap of the steering PWM slice, after the hardware has loaded the buffered values
    Wrap on_wrap() {
        position_ += running_.direction;  // the pulse that just finished, if any
        running_ = buffered_;
        buffered_ = idle();
        if (!stopped_ && holdoff_ > 0) {
            holdoff_--;
        } else if (!stopped_) {
            const SteeringMotion::Step step = jog_direction_ != 0
                ? SteeringMotion::Step{jog_direction_, 2000, 0.0f, 0.0f}
                : motion_.next_step();
            if (step.direction != 0) {
                if (jog_direction_ == 0) motion_.step_taken(step);
                buffered_ = pulse(step.interval_us, step.direction);
            }
        }
        return {buffered_, running_.direction};
    }

    // Homing uses a conservative fixed 500 Hz pulse rate, independent of soft limits.
    void jog(int direction) { jog_direction_ = direction; }

    // Only rebase after stop() and after the hardware pipeline has drained.
    void zero() {
        position_ = 0;
        motion_.zero();
        jog_direction_ = 0;
    }

    // stops planning; steps already handed to the PWM still go out
    void stop() {
        stopped_ = true;
        motion_.stop();
    }

    // starts planning again after holdoff_periods idle periods, e.g. so a driver that was just
    // enabled can settle before the first pulse. does nothing if already running, so it's safe
    // to call on every command without restarting the holdoff
    void resume(uint32_t holdoff_periods = 0) {
        if (stopped_) {
            stopped_ = false;
            holdoff_ = holdoff_periods;
        }
    }

    // stopped, and no pulse is left in the PWM: the driver can be disabled without losing steps
    bool drained() const { return stopped_ && running_.direction == 0 && buffered_.direction == 0; }

    // the period both PWM registers should hold before the slice starts
    Period idle() const { return {static_cast<uint16_t>(idle_us_ - 1), idle_us_, 0}; }

    int32_t position() const { return position_; }

private:
    Period pulse(uint32_t interval_us, int direction) const {
        const auto length = static_cast<uint16_t>(std::clamp<uint32_t>(interval_us, 2u * pulse_us_, max_us_));
        return {static_cast<uint16_t>(length - 1), static_cast<uint16_t>(length - pulse_us_), direction};
    }

    SteeringMotion& motion_;
    uint16_t pulse_us_;
    uint16_t idle_us_;
    uint16_t max_us_;
    Period running_ = idle();   // the period the PWM is in now
    Period buffered_ = idle();  // the period the PWM loads at the next wrap
    int32_t position_ = 0;
    int jog_direction_ = 0;
    bool stopped_ = true;
    uint32_t holdoff_ = 0;  // idle periods left before planning resumes
};
