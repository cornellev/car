#pragma once

#include <cstdint>

#include "chuds/chuds.hpp"
#include "common/interval.hpp"

namespace cev {

// if m is a StatusBlink naming node, set blink's period from it
// call it on every received message
inline void apply_status_blink(const chuds::Message& m, std::uint8_t node, Interval& blink) {
    if (const auto b = chuds::catalog_body<chuds::StatusBlink>(m);
        b && b->node == node && b->half_period_ms != 0) {
        blink.period_ms = b->half_period_ms;
    }
}

}  // namespace cev
