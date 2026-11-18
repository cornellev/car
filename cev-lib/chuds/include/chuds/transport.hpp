#pragma once

#include <concepts>
#include <cstdint>
#include <expected>

#include "chuds/frame.hpp"

namespace chuds {

// why a send failed
enum class TxError : std::uint8_t {
    // tx queue full, retry later
    QueueFull,
    // controller is bus-off
    BusOff,
    // bad frame or driver fault
    Error,
};

// why a recv returned no frame
enum class RxError : std::uint8_t {
    // nothing waiting, not a fault
    Empty,
    // rx queue overflowed, frames dropped
    Overflow,
    // bus is off, a fault not silence
    BusOff,
    // driver fault or closed transport
    Error,
    // a classic, remote, extended, or out-of-range frame, never a chuds frame
    ForeignFrame,
    // the frame arrived but did not decode, reported by decode and chuds::Bus::recv
    NonStandardId,
    UnknownClass,
    BadLength,
    UnknownType,
    BodyOverrun,
};

// success means queued, not yet on the wire
using TxResult = std::expected<void, TxError>;

using RxResult = std::expected<CanFrame, RxError>;

// whether an rx error is worth reporting: the bus or driver had a problem
// Overflow and BusOff can recover, so this does not mean stop receiving
[[nodiscard]] constexpr bool is_fault(RxError e) {
    switch (e) {
        case RxError::Overflow:
        case RxError::BusOff:
        case RxError::Error: return true;
        case RxError::Empty:
        case RxError::ForeignFrame:
        case RxError::NonStandardId:
        case RxError::UnknownClass:
        case RxError::BadLength:
        case RxError::UnknownType:
        case RxError::BodyOverrun: return false;
    }
    return true;
}

// whether a tx error is worth reporting
// QueueFull only means retry later
[[nodiscard]] constexpr bool is_fault(TxError e) {
    switch (e) {
        case TxError::BusOff:
        case TxError::Error: return true;
        case TxError::QueueFull: return false;
    }
    return true;
}

// the controller's standing on the bus, set by its own error counters
enum class BusState : std::uint8_t {
    // normal operation
    Active,
    // many recent errors, it still sends but no longer flags other nodes' errors
    Passive,
    // too many transmit errors, it has left the bus until it recovers
    Off,
};

// the controller's error state right now
// a node alone on the bus goes passive, with tx_errors climbing, because nothing acknowledges it
struct BusStatus {
    BusState state{};
    std::uint8_t tx_errors{};
    std::uint8_t rx_errors{};
};

// the interface between chuds and a driver
// send and recv report explicit outcomes so a dead bus is not mistaken for quiet
template <typename T>
concept Transport = requires(T t, const CanFrame& f) {
    { t.send(f) } -> std::same_as<TxResult>;
    { t.recv() } -> std::same_as<RxResult>;
};

}  // namespace chuds
