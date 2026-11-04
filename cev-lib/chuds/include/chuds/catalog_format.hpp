#pragma once

#include <cstdio>
#include <span>
#include <utility>

#include "chuds/catalog.hpp"
#include "chuds/message.hpp"

namespace chuds {

// write one readable line for m into out, truncated to fit
// returns the length snprintf would have written
inline int format_message(const Message& m, std::span<char> out) {
    if (const auto b = read_stop(m)) {
        return std::snprintf(out.data(), out.size(), "stop severity %d from 0x%02x", b->severity,
                             b->sender);
    }
    if (const auto b = catalog_body<BodyState>(m)) {
        return std::snprintf(out.data(), out.size(), "body state 0x%02x", b->bits);
    }
    if (const auto b = catalog_body<front_aux::Telemetry>(m)) {
        return std::snprintf(out.data(), out.size(), "front_aux rpm %d/%d steering %d", b->rpm_left,
                             b->rpm_right, b->steering);
    }
    if (const auto b = catalog_body<StatusBlink>(m)) {
        return std::snprintf(out.data(), out.size(), "status blink %d ms to 0x%02x",
                             b->half_period_ms, m.subaddress);
    }
    if (const auto b = catalog_body<back_aux::Telemetry>(m)) {
        return std::snprintf(out.data(), out.size(), "back_aux rpm %d/%d brake %d", b->rpm_left,
                             b->rpm_right, b->brake);
    }
    if (const auto b = catalog_body<joulemeter::Telemetry>(m)) {
        return std::snprintf(out.data(), out.size(), "joulemeter %.2f V %.2f A %.1f W %.1f J",
                             b->voltage, b->current, b->power, b->energy);
    }
    // not in the catalog, or its body size does not match
    return std::snprintf(
        out.data(), out.size(), "class %d subaddress 0x%02x type %d, %d body bytes",
        std::to_underlying(m.cls), m.subaddress, std::to_underlying(m.type), m.body_len);
}

// print one line for m to stdout
inline void print_message(const Message& m) {
    char line[96];
    format_message(m, line);
    std::printf("%s\n", line);
}

}  // namespace chuds
