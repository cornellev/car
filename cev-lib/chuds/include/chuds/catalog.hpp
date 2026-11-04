#pragma once

#include <array>
#include <cstdint>

#include "chuds/ids.hpp"

namespace chuds {

// command subaddress of the leader's broadcast body state, can id 0x108
constexpr std::uint8_t kBodyStateId = 0x08;

// desired state of the car's body outputs, broadcast by the leader
// resent on change and periodically, each node drives only the outputs it has
struct BodyState {
    static constexpr MsgClass kClass          = MsgClass::Command;
    static constexpr std::uint8_t kSubaddress = kBodyStateId;

    // bits 6-7 are reserved and sent as 0
    static constexpr std::uint8_t kLeftTurn   = 1 << 0;
    static constexpr std::uint8_t kRightTurn  = 1 << 1;
    static constexpr std::uint8_t kHeadlights = 1 << 2;
    static constexpr std::uint8_t kHorn       = 1 << 3;
    static constexpr std::uint8_t kWiper      = 1 << 4;
    // turn lamps light only while set, the leader toggles it so every node blinks in phase
    static constexpr std::uint8_t kBlinkPhase = 1 << 5;

    std::uint8_t bits{};
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(BodyState) == 1);

}  // namespace chuds

namespace chuds::front_aux {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x10;

// telemetry body, published once per cycle
struct Telemetry {
    static constexpr MsgClass kClass          = MsgClass::Telemetry;
    static constexpr std::uint8_t kSubaddress = kNodeId;

    std::uint16_t rpm_left{};   // wheel edges in the last window
    std::uint16_t rpm_right{};  // wheel edges in the last window
    std::uint16_t steering{};   // raw adc, 0-4095
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(Telemetry) == 6);

}  // namespace chuds::front_aux

namespace chuds::back_aux {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x20;

// telemetry body, published once per cycle
struct Telemetry {
    static constexpr MsgClass kClass          = MsgClass::Telemetry;
    static constexpr std::uint8_t kSubaddress = kNodeId;

    std::uint16_t rpm_left{};   // wheel edges in the last window
    std::uint16_t rpm_right{};  // wheel edges in the last window
    std::uint16_t brake{};      // raw adc, 0-4095
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(Telemetry) == 6);

}  // namespace chuds::back_aux

namespace chuds::joulemeter {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x30;

// telemetry body, published every report period
struct Telemetry {
    static constexpr MsgClass kClass          = MsgClass::Telemetry;
    static constexpr std::uint8_t kSubaddress = kNodeId;

    float voltage{};  // bus volts, averaged over the report period
    float current{};  // bus amps, averaged over the report period
    float power{};    // watts, averaged over the report period
    float energy{};   // joules since boot
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(Telemetry) == 16);

}  // namespace chuds::joulemeter

namespace chuds::devboard {

// this node's id: command target, shared by every dev board running the bridge
constexpr std::uint8_t kNodeId = 0x40;

}  // namespace chuds::devboard

namespace chuds {

// command body sent to one node, sets its status LED's blink half-period
// nodes ignore 0
struct StatusBlink {
    static constexpr MsgClass kClass = MsgClass::Command;
    // every node that takes it
    static constexpr std::array<std::uint8_t, 4> kSubaddresses{
        front_aux::kNodeId, back_aux::kNodeId, joulemeter::kNodeId, devboard::kNodeId};

    std::uint16_t half_period_ms{};
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(StatusBlink) == 2);

}  // namespace chuds
