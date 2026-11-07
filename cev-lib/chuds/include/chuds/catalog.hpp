#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "chuds/ids.hpp"

namespace chuds {

// desired state of the car's body outputs, broadcast by the leader
// resent on change and periodically, each node drives only the outputs it has
struct BodyState {
    // can id 0x108
    static constexpr CanId kId{Band::Command, 0x08};

    // bits 6-7 are reserved and sent as 0
    static constexpr std::uint8_t kLeftTurn   = 1 << 0;
    static constexpr std::uint8_t kRightTurn  = 1 << 1;
    static constexpr std::uint8_t kHeadlights = 1 << 2;
    static constexpr std::uint8_t kHorn       = 1 << 3;
    static constexpr std::uint8_t kWiper      = 1 << 4;
    // turn lamps light only while set, the leader toggles it so every node blinks in phase
    static constexpr std::uint8_t kBlinkPhase = 1 << 5;

    std::uint8_t bits{};
};
static_assert(sizeof(BodyState) == 1);

}  // namespace chuds

namespace chuds::front_aux {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x10;

// telemetry body, published once per cycle
struct Telemetry {
    // can id 0x210
    static constexpr CanId kId{Band::Telemetry, kNodeId};

    std::uint16_t rpm_left{};   // wheel edges in the last window
    std::uint16_t rpm_right{};  // wheel edges in the last window
    std::uint16_t steering{};   // raw adc, 0-4095
};
static_assert(sizeof(Telemetry) == 6);

}  // namespace chuds::front_aux

namespace chuds::back_aux {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x20;

// telemetry body, published once per cycle
struct Telemetry {
    // can id 0x220
    static constexpr CanId kId{Band::Telemetry, kNodeId};

    std::uint16_t rpm_left{};   // wheel edges in the last window
    std::uint16_t rpm_right{};  // wheel edges in the last window
    std::uint16_t brake{};      // raw adc, 0-4095
};
static_assert(sizeof(Telemetry) == 6);

}  // namespace chuds::back_aux

namespace chuds::joulemeter {

// this node's id: telemetry source
constexpr std::uint8_t kNodeId = 0x30;

// telemetry body, published every report period
struct Telemetry {
    // can id 0x230
    static constexpr CanId kId{Band::Telemetry, kNodeId};

    float voltage{};  // bus volts, averaged over the report period
    float current{};  // bus amps, averaged over the report period
    float power{};    // watts, averaged over the report period
    float energy{};   // joules since boot
};
static_assert(sizeof(Telemetry) == 16);

}  // namespace chuds::joulemeter

namespace chuds::devboard {

// this node's id: command target, shared by every dev board running the bridge
constexpr std::uint8_t kNodeId = 0x40;

}  // namespace chuds::devboard

namespace chuds {

// command body broadcast to every node, sets the status LED's blink half-period of the node it
// names, nodes ignore it unless node is their own id, and ignore 0
struct StatusBlink {
    // can id 0x109, tentative until the id table is ratified
    static constexpr CanId kId{Band::Command, 0x09};

    std::uint8_t node{};
    std::uint8_t reserved{};  // sent as 0
    std::uint16_t half_period_ms{};
};
static_assert(sizeof(StatusBlink) == 4);

// a list of types, to check them all at once
template <class... Ts>
struct TypeList {};

// the nodes that own a block of Telemetry and Status ids, 0xNM for node N's message M
inline constexpr std::array kNodes{front_aux::kNodeId, back_aux::kNodeId, joulemeter::kNodeId};

// every struct above that owns an id, add new ones here
// STOP is not listed because its id varies with severity
using AllMessages = TypeList<BodyState, front_aux::Telemetry, back_aux::Telemetry,
                             joulemeter::Telemetry, StatusBlink>;

namespace detail {

[[nodiscard]] constexpr bool in_node_block(std::uint8_t subaddress) {
    for (const auto node : kNodes) {
        if ((subaddress & 0xF0) == node) {
            return true;
        }
    }
    return false;
}

// no two structs share an id
template <class... Ts>
[[nodiscard]] consteval bool ids_unique(TypeList<Ts...>) {
    constexpr std::array<std::uint16_t, sizeof...(Ts)> ids{Ts::kId.raw()...};

    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            if (ids[i] == ids[j]) {
                return false;
            }
        }
    }
    return true;
}

// each Telemetry and Status id sits in a registered node's block
template <class... Ts>
[[nodiscard]] consteval bool ids_in_node_blocks(TypeList<Ts...>) {
    return (((Ts::kId.cls() != Band::Telemetry && Ts::kId.cls() != Band::Status) ||
             in_node_block(Ts::kId.subaddress())) &&
            ...);
}

}  // namespace detail

static_assert(detail::ids_unique(AllMessages{}), "two messages share a CAN id");
static_assert(detail::ids_in_node_blocks(AllMessages{}),
              "a Telemetry or Status id is outside every registered node's block");

}  // namespace chuds
