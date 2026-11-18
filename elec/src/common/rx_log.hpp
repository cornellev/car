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
        const bool body_state =
            m.cls == chuds::BodyState::kClass && m.subaddress == chuds::BodyState::kSubaddress;
        const bool to_me = m.cls == chuds::MsgClass::Command && m.subaddress == node;
        if (!body_state && !to_me && m.cls != chuds::MsgClass::Emergency) {
            return;
        }
        if (body_state) {
            if (auto b = chuds::body_as<chuds::BodyState>(m)) {
                b->set_blink_phase(false);
                if (b->raw() == last_body) {
                    return;
                }
                last_body = b->raw();
            }
        }
        std::printf("rx ");
        chuds::print_message(m);
    }
};

}  // namespace cev
