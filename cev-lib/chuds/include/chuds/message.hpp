#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/transport.hpp"

namespace chuds {

// why a STOP was sent
enum class StopReason : std::uint8_t {
    Unknown   = 0,
    Operator  = 1,  // e-stop button or base station
    CommsLost = 2,  // VCU lost its command link
    NodeFault = 3,  // a node reported a fault
};

// payload is just the body, so the id says what it is
// body_len is the frame length, which CAN-FD pads to a valid size
inline constexpr std::size_t kMaxBody = kMaxFdPayload;

// a decoded CHUDS message: the id fields (class + subaddress) and the typed body
struct Message {
    Band cls{Band::Telemetry};
    std::uint8_t subaddress{};
    std::array<std::uint8_t, kMaxBody> body{};
    std::uint8_t body_len{};

    // a view over the body bytes, without the trailing capacity
    [[nodiscard]] constexpr std::span<const std::uint8_t> body_view() const {
        return std::span(body).first(std::min<std::size_t>(body_len, body.size()));
    }
};

// structs are sent as raw bytes, so the host must be little endian
static_assert(std::endian::native == std::endian::little);
static_assert(std::numeric_limits<float>::is_iec559);

namespace detail {

// compiles only if T has no padding and no pointer members
// reading a padding byte, or bit_cast of a pointer, is not allowed at compile time
template <class T>
consteval bool has_no_padding_or_pointers() {
    for (const auto byte : std::bit_cast<std::array<unsigned char, sizeof(T)>>(T{})) {
        static_cast<void>(byte);
    }
    return true;
}

// T owns its CAN id, as `static constexpr CanId kId{Band, sub};`
template <class T>
concept HasWireId = requires { requires std::same_as<std::remove_cv_t<decltype(T::kId)>, CanId>; };

// satisfied when has_no_padding_or_pointers<T>() is a constant expression
template <class T>
concept NoPaddingOrPointers =
    requires { typename std::bool_constant<has_no_padding_or_pointers<T>()>; };

}  // namespace detail

// a struct that make_message and body_as can bit_cast onto the wire
// bool, bit-fields, and enums without a fixed underlying type pass these checks but are unsafe
template <class T>
concept WireBody =
    detail::HasWireId<T> && std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T> &&
    sizeof(T) <= kMaxBody && detail::NoPaddingOrPointers<T>;

namespace detail {

// one message per WireBody rule, then WireBody itself so a rule missing here still fails
template <class T>
constexpr void check_wire_body() {
    static_assert(HasWireId<T>,
                  "wire body must opt in by owning its id with `static constexpr CanId kId{Band, "
                  "sub};` after checking it has no bool, bit-field, or enum without a fixed "
                  "underlying type (chuds/docs/serialization.md)");
    static_assert(std::is_trivially_copyable_v<T>, "body must be trivially copyable");
    static_assert(std::is_standard_layout_v<T>, "body must be standard-layout");
    static_assert(sizeof(T) <= kMaxBody, "body is larger than the CAN-FD payload");
    static_assert(NoPaddingOrPointers<T>,
                  "body must have no padding or pointers, and a constexpr default constructor");
    static_assert(WireBody<T>);
}

}  // namespace detail

// build a message whose body is a trivially-copyable struct
// the struct's bytes are the wire body, little-endian
// the message goes out on the id the struct owns
// returns nullopt if the class is invalid
template <class T>
[[nodiscard]] constexpr std::optional<Message> make_message(const T& value) {
    // ensure the struct can be sent over wire
    detail::check_wire_body<T>();

    if (!is_valid(T::kId.cls())) {
        return std::nullopt;
    }

    const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);

    Message m{.cls = T::kId.cls(), .subaddress = T::kId.subaddress()};
    std::copy_n(bytes.begin(), bytes.size(), m.body.begin());
    m.body_len = round_up_fd_length(static_cast<std::uint8_t>(sizeof(T)));

    return m;
}

// read the body of a message back as a struct
// returns nullopt unless the id is T::kId and the frame length is the padded size of T
template <class T>
[[nodiscard]] constexpr std::optional<T> body_as(const Message& m) {
    // ensure the struct can be sent over wire
    detail::check_wire_body<T>();

    if (CanId(m.cls, m.subaddress) != T::kId ||
        m.body_len != round_up_fd_length(static_cast<std::uint8_t>(sizeof(T)))) {
        return std::nullopt;
    }

    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::copy_n(m.body.begin(), sizeof(T), bytes.begin());

    return std::bit_cast<T>(bytes);
}

// m's body as B when m is on B's id, see catalog.hpp
// body_as checks the id and the padded length, so this is the same read
template <class B>
[[nodiscard]] constexpr std::optional<B> catalog_body(const Message& m) {
    return body_as<B>(m);
}

// make a STOP message
// transmits an emergency class with a subaddress as the severity (0 is the most severe)
// body contains [sender][reason]
// never fails, so a stop can always be sent
// only the VCU (pi/jetson) sends STOP, since each CAN id must have exactly one transmitter
[[nodiscard]] constexpr Message make_stop(std::uint8_t severity, std::uint8_t sender,
                                          StopReason reason) {
    Message m{.cls = Band::Emergency, .subaddress = severity};
    m.body[0]  = sender;
    m.body[1]  = std::to_underlying(reason);
    m.body_len = round_up_fd_length(2);

    return m;
}

// a decoded STOP: severity from the subaddress, then [sender][reason]
struct StopView {
    std::uint8_t severity{};
    std::uint8_t sender{};
    StopReason reason{};
};

// read a STOP, or nullopt if the message is not a well-formed STOP
[[nodiscard]] constexpr std::optional<StopView> read_stop(const Message& m) {
    if (m.cls != Band::Emergency || m.body_len != round_up_fd_length(2)) {
        return std::nullopt;
    }

    StopView s{};
    s.severity = m.subaddress;
    s.sender   = m.body[0];
    s.reason   = static_cast<StopReason>(m.body[1]);

    return s;
}

// id = class + subaddress, payload = body
[[nodiscard]] constexpr std::optional<CanFrame> encode(const Message& m) {
    if (!is_valid(m.cls) || m.body_len > kMaxBody) {
        return std::nullopt;
    }

    CanFrame f{};
    f.id = CanId(m.cls, m.subaddress);

    std::copy_n(m.body.begin(), m.body_len, f.data.begin());

    // round up to a valid CAN-FD length
    // the unused tail is already zero from init
    f.len = round_up_fd_length(m.body_len);
    return f;
}

// fails only with NonStandardId, UnknownClass, or BadLength
[[nodiscard]] constexpr std::expected<Message, RxError> decode(const CanFrame& f) {
    if (!f.id.is_standard()) {
        return std::unexpected(RxError::NonStandardId);
    }

    if (!is_valid(f.id.cls())) {
        return std::unexpected(RxError::UnknownClass);
    }

    if (!is_valid_fd_len(f.len)) {
        return std::unexpected(RxError::BadLength);
    }

    Message m{
        .cls        = f.id.cls(),
        .subaddress = f.id.subaddress(),
    };

    std::copy_n(f.data.begin(), f.len, m.body.begin());
    m.body_len = f.len;

    return m;
}

}  // namespace chuds
