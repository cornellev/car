#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "hardware/gpio.h"
#include "pico/time.h"

namespace cev {

// wheel-speed edge counters, shared with the gpio irq
inline std::atomic<std::uint32_t> g_rpm_left{0};
inline std::atomic<std::uint32_t> g_rpm_right{0};
inline unsigned g_rpm_left_pin{};
inline unsigned g_rpm_right_pin{};
inline std::uint32_t g_rpm_prev_us{};

inline void rpm_on_edge(unsigned gpio, std::uint32_t /*events*/) {
    if (gpio == g_rpm_left_pin) {
        g_rpm_left.fetch_add(1, std::memory_order_relaxed);
    } else if (gpio == g_rpm_right_pin) {
        g_rpm_right.fetch_add(1, std::memory_order_relaxed);
    }
}

// count rising edges on two wheel-speed inputs
// claims the core's single gpio irq callback
// inputs expect a pnp sensor driving 12v through the board's divider, which also holds the pin low
inline void rpm_init(unsigned left_pin, unsigned right_pin) {
    g_rpm_left_pin  = left_pin;
    g_rpm_right_pin = right_pin;
    for (const unsigned pin : {left_pin, right_pin}) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }
    g_rpm_prev_us = time_us_32();
    gpio_set_irq_enabled_with_callback(left_pin, GPIO_IRQ_EDGE_RISE, true, &rpm_on_edge);
    gpio_set_irq_enabled(right_pin, GPIO_IRQ_EDGE_RISE, true);
}

struct RpmSample {
    std::uint16_t left;
    std::uint16_t right;
    std::uint32_t ts;
};

// Whole RPM, rounded to nearest and saturated to the wire field's range.
// There are 32 rising edges per wheel revolution.
[[nodiscard]] constexpr std::uint16_t pulses_to_rpm(std::uint32_t pulses,
                                                    std::uint32_t elapsed_us) {
    if (elapsed_us == 0) {
        return 0;
    }
    const double rpm =
        static_cast<double>(pulses) * 60'000'000.0 / (32.0 * static_cast<double>(elapsed_us));
    return rpm >= 65535.0 ? 65535 : static_cast<std::uint16_t>(rpm + 0.5);
}

// Sample both counters and convert using the actual window, then reset.
[[nodiscard]] inline RpmSample rpm_take() {
    const auto now                 = time_us_32();
    const auto left                = g_rpm_left.exchange(0, std::memory_order_relaxed);
    const auto right               = g_rpm_right.exchange(0, std::memory_order_relaxed);
    const std::uint32_t elapsed_us = now - g_rpm_prev_us;  // unsigned subtraction handles rollover
    g_rpm_prev_us                  = now;
    return {pulses_to_rpm(left, elapsed_us), pulses_to_rpm(right, elapsed_us), now};
}

}  // namespace cev
