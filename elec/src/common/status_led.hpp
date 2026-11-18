#pragma once

#include <cstdint>

#include "chuds/chuds.hpp"
#include "common/interval.hpp"
#include "hardware/gpio.h"

namespace cev {

// one or more status-LED pins blinked together at a settable half-period
// a StatusBlink command aimed at this node sets the period, a zero period is ignored
class StatusLed {
   public:
    StatusLed(unsigned pin, std::uint32_t half_period_ms) : blink_{half_period_ms} {
        init(1U << pin);
    }

    template <class Pins>
    StatusLed(const Pins& pins, std::uint32_t half_period_ms) : blink_{half_period_ms} {
        std::uint32_t mask = 0;
        for (const unsigned pin : pins) {
            mask |= 1U << pin;
        }
        init(mask);
    }

    // toggle the pins once per blink period
    void tick() {
        if (!blink_.due()) {
            return;
        }
        on_ = !on_;
        gpio_put_masked(pins_, on_ ? pins_ : 0);
    }

    // adopt a StatusBlink period aimed at node, called on every received message
    void apply_status_blink(const chuds::Message& m, std::uint8_t node) {
        if (m.subaddress != node) {
            return;
        }
        if (const auto b = chuds::catalog_body<chuds::StatusBlink>(m);
            b && b->half_period_ms != 0) {
            blink_.period_ms = b->half_period_ms;
        }
    }

   private:
    void init(std::uint32_t mask) {
        pins_ = mask;
        gpio_init_mask(pins_);
        gpio_set_dir_out_masked(pins_);
        gpio_put_masked(pins_, 0);
    }

    std::uint32_t pins_{};
    Interval blink_;
    bool on_{};
};

}  // namespace cev
