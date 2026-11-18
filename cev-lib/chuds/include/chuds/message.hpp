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
#include <string_view>
#include <type_traits>
#include <utility>

#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/transport.hpp"

namespace chuds {

enum class MsgType : std::uint8_t {
    Update    = 0,
    Heartbeat = 1,
    Stop      = 2,
    Action    = 3,
    Dummy     = 4,
};

// a type value is defined only over Update..Dummy
[[nodiscard]] constexpr bool is_valid(MsgType type) { return type <= MsgType::Dummy; }

// payload is [type][body_len][body...]
// the explicit length recovers the true body size despite CAN-FD DLC padding
inline constexpr std::size_t kTypeOffset    = 0;
inline constexpr std::size_t kBodyLenOffset = 1;
inline constexpr std::size_t kHeaderLen     = 2;
inline constexpr std::size_t kMaxBody       = kMaxFdPayload - kHeaderLen;
static_assert(kHeaderLen + kMaxBody == kMaxFdPayload);

// read the header fields out of a frame's data
[[nodiscard]] constexpr MsgType frame_type(const CanFrame& f) {
    return static_cast<MsgType>(f.data[kTypeOffset]);
}

[[nodiscard]] constexpr std::uint8_t frame_body_len(const CanFrame& f) {
    return f.data[kBodyLenOffset];
}

// a decoded CHUDS message: the id fields (class + subaddress) and the typed body
struct Message {
    MsgClass cls{MsgClass::Telemetry};
    std::uint8_t subaddress{};
    MsgType type{MsgType::Dummy};
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

template <class T>
concept HasWireBodyMarker = requires { requires T::is_chuds_wire_body; };

// satisfied when has_no_padding_or_pointers<T>() is a constant expression
template <class T>
concept NoPaddingOrPointers =
    requires { typename std::bool_constant<has_no_padding_or_pointers<T>()>; };

}  // namespace detail

// a struct that make_message and body_as can bit_cast onto the wire
// bool, bit-fields, and enums without a fixed underlying type pass these checks but are unsafe
template <class T>
concept WireBody =
    detail::HasWireBodyMarker<T> && std::is_trivially_copyable_v<T> &&
    std::is_standard_layout_v<T> && sizeof(T) <= kMaxBody && detail::NoPaddingOrPointers<T>;

namespace detail {

// one message per WireBody rule, then WireBody itself so a rule missing here still fails
template <class T>
constexpr void check_wire_body() {
    static_assert(HasWireBodyMarker<T>,
                  "wire body must opt in with `static constexpr bool is_chuds_wire_body = true;` "
                  "after checking it has no bool, bit-field, or enum without a fixed underlying "
                  "type (chuds/docs/serialization.md)");
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
// returns nullopt if the class or type is invalid
template <class T>
[[nodiscard]] constexpr std::optional<Message> make_message(MsgClass cls, std::uint8_t sub,
                                                            MsgType type, const T& value) {
    // ensure the struct can be sent over wire
    detail::check_wire_body<T>();

    if (!is_valid(cls) || !is_valid(type)) {
        return std::nullopt;
    }

    const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);

    Message m{.cls = cls, .subaddress = sub, .type = type};
    std::copy_n(bytes.begin(), bytes.size(), m.body.begin());
    m.body_len = sizeof(T);

    return m;
}

// read the body of a message back as a struct
// returns nullopt if the body size is not exactly sizeof(T)
template <class T>
[[nodiscard]] constexpr std::optional<T> body_as(const Message& m) {
    // ensure the struct can be sent over wire
    detail::check_wire_body<T>();

    if (m.body_len != sizeof(T)) {
        return std::nullopt;
    }

    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::copy_n(m.body.begin(), sizeof(T), bytes.begin());

    return std::bit_cast<T>(bytes);
}

// a body always sent with kClass to its one kSubaddress
template <class B>
concept SingleAddressBody = requires {
    { B::kClass } -> std::convertible_to<MsgClass>;
    { B::kSubaddress } -> std::convertible_to<std::uint8_t>;
};

// a body sent with kClass to any one of its kSubaddresses
template <class B>
concept MultiAddressBody = requires {
    { B::kClass } -> std::convertible_to<MsgClass>;
    std::ranges::find(B::kSubaddresses, std::uint8_t{});
};

// a body whose class and addresses are part of its type, see catalog.hpp
template <class B>
concept CatalogBody = SingleAddressBody<B> || MultiAddressBody<B>;

// whether subaddress is one of B's addresses
template <CatalogBody B>
[[nodiscard]] constexpr bool is_address_of(std::uint8_t subaddress) {
    if constexpr (SingleAddressBody<B>) {
        return subaddress == B::kSubaddress;
    } else {
        return std::ranges::find(B::kSubaddresses, subaddress) != B::kSubaddresses.end();
    }
}

// m's body as B when m is sent to one of B's addresses
// a command counts only as an Update, since boards act on nothing else
template <CatalogBody B>
[[nodiscard]] constexpr std::optional<B> catalog_body(const Message& m) {
    if (m.cls != B::kClass || !is_address_of<B>(m.subaddress) ||
        (B::kClass == MsgClass::Command && m.type != MsgType::Update)) {
        return std::nullopt;
    }
    return body_as<B>(m);
}

// make a STOP message
// transmits an emergency class with a subaddress as the severity (0 is the most severe)
// body contains [sender][detail...]
// overly long detail is truncated to prevent failures
// never fails, so a stop can always be sent
// only the VCU (pi/jetson) sends STOP, since each CAN id must have exactly one transmitter
[[nodiscard]] constexpr Message make_stop(std::uint8_t severity, std::uint8_t sender,
                                          std::string_view detail) {
    constexpr std::size_t kMaxDetail = kMaxBody - 1;
    const std::size_t n              = detail.size() < kMaxDetail ? detail.size() : kMaxDetail;

    Message m{.cls = MsgClass::Emergency, .subaddress = severity, .type = MsgType::Stop};
    m.body[0] = sender;
    std::copy_n(detail.begin(), n, m.body.begin() + 1);
    m.body_len = static_cast<std::uint8_t>(n + 1);

    return m;
}

// a decoded STOP: severity from the subaddress, then [sender][detail...]
struct StopView {
    std::uint8_t severity{};
    std::uint8_t sender{};
    std::span<const std::uint8_t> detail{};
};

// read a STOP, or nullopt if the message is not a well-formed STOP
[[nodiscard]] constexpr std::optional<StopView> read_stop(const Message& m) {
    if (m.cls != MsgClass::Emergency || m.type != MsgType::Stop || m.body_len < 1) {
        return std::nullopt;
    }

    StopView s{};
    s.severity = m.subaddress;
    s.sender   = m.body[0];
    s.detail   = m.body_view().subspan(1);

    return s;
}

// the detail span points into m, so a temporary message would leave it dangling
std::optional<StopView> read_stop(const Message&& m) = delete;

// id = class + subaddress, payload = [type][body_len][body...]
[[nodiscard]] constexpr std::optional<CanFrame> encode(const Message& m) {
    if (!is_valid(m.cls) || !is_valid(m.type) || m.body_len > kMaxBody) {
        return std::nullopt;
    }

    CanFrame f{};
    f.id                   = CanId(m.cls, m.subaddress);
    f.data[kTypeOffset]    = std::to_underlying(m.type);
    f.data[kBodyLenOffset] = m.body_len;

    std::copy_n(m.body.begin(), m.body_len, f.data.begin() + kHeaderLen);

    // round up to a valid CAN-FD length
    // the unused tail is already zero from init
    f.len = round_up_fd_length(static_cast<std::uint8_t>(kHeaderLen + m.body_len));
    return f;
}

// fails only with NonStandardId, UnknownClass, BadLength, UnknownType, or BodyOverrun
[[nodiscard]] constexpr std::expected<Message, RxError> decode(const CanFrame& f) {
    if (!f.id.is_standard()) {
        return std::unexpected(RxError::NonStandardId);
    }

    if (!is_valid(f.id.cls())) {
        return std::unexpected(RxError::UnknownClass);
    }

    if (f.len < kHeaderLen || !is_valid_fd_len(f.len)) {
        return std::unexpected(RxError::BadLength);
    }

    if (!is_valid(frame_type(f))) {
        return std::unexpected(RxError::UnknownType);
    }

    const std::uint8_t body_len = frame_body_len(f);

    // the claimed body must fit within the frame
    if (kHeaderLen + body_len > f.len) {
        return std::unexpected(RxError::BodyOverrun);
    }

    Message m{
        .cls        = f.id.cls(),
        .subaddress = f.id.subaddress(),
        .type       = frame_type(f),
    };

    std::copy_n(f.data.begin() + kHeaderLen, body_len, m.body.begin());
    m.body_len = body_len;

    return m;
}

}  // namespace chuds
