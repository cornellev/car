#pragma once

#include <cstdint>
#include <cstdio>

#include "chuds/chuds.hpp"

namespace cev {

// prints the commands this node receives on usb serial, so a bench test can see them arrive
// call log() on every received message
struct RxLog {
    std::uint8_t node{};
    // body state bits last printed, without the blink phase, since the leader resends it constantly
    int last_body{-1};

    void log(const chuds::Message& m) {
        const bool body_state = chuds::catalog_body<chuds::BodyState>(m).has_value();
        const auto blink      = chuds::catalog_body<chuds::StatusBlink>(m);
        const bool to_me      = (m.cls == chuds::Band::Command && m.subaddress == node) ||
                           (blink && blink->node == node);
        if (!body_state && !to_me && m.cls != chuds::Band::Emergency) {
            return;
        }
        if (body_state) {
            if (const auto b = chuds::body_as<chuds::BodyState>(m)) {
                const int bits = b->bits & ~chuds::BodyState::kBlinkPhase;
                if (bits == last_body) {
                    return;
                }
                last_body = bits;
            }
        }
        std::printf("rx ");
        chuds::print_message(m);
    }
};

}  // namespace cev
