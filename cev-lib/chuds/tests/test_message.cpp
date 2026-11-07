#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <expected>
#include <utility>

#include "chuds/catalog.hpp"
#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/message.hpp"
#include "chuds/transport.hpp"

using namespace chuds;

namespace {
struct WheelSpeed {
    static constexpr CanId kId{Band::Telemetry, 0x10};
    std::uint16_t rpm;
};
static_assert(sizeof(WheelSpeed) == 2);

enum class Mode : std::uint8_t { Off, On };
struct Mixed {
    std::uint32_t a{};
    std::int16_t b{};
    Mode mode{};
    std::uint8_t c{};
    float d{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct Pair {
    std::uint16_t a{};
    std::uint16_t b{};
};
struct Nested {
    Pair pair{};
    std::uint32_t c{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct Array {
    std::array<std::uint16_t, 3> values{};
    std::uint16_t d{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};

static_assert(WireBody<WheelSpeed>);
static_assert(WireBody<Mixed>);
static_assert(WireBody<Nested>);
static_assert(WireBody<Array>);

// every body below except Unmarked opts in, so each fails on the rule its name gives
struct Unmarked {
    std::uint16_t a{};
};
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
struct NotTriviallyCopyable {
    std::uint16_t a{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
    NotTriviallyCopyable() = default;
    // NOLINTNEXTLINE(modernize-use-equals-default)
    NotTriviallyCopyable(const NotTriviallyCopyable& other) : a(other.a) {}
};
class NotStandardLayout {
   public:
    std::uint16_t a{};
    static constexpr CanId kId{Band::Telemetry, 0x10};

   private:
    [[maybe_unused]] std::uint16_t b{};
};
struct TooBig {
    std::array<std::uint8_t, kMaxBody + 1> bytes{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct MidPadding {
    std::uint8_t a{};
    float b{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct TailPadding {
    float a{};
    std::uint8_t b{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct NestedPadding {
    MidPadding inner{};
    std::uint32_t c{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct ArrayOfPadding {
    std::array<MidPadding, 2> values{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct Pointer {
    const char* text{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};
struct NestedPointer {
    Pointer inner{};
    static constexpr CanId kId{Band::Telemetry, 0x10};
};

static_assert(!WireBody<Unmarked>);
static_assert(!WireBody<NotTriviallyCopyable>);
static_assert(!WireBody<NotStandardLayout>);
static_assert(!WireBody<TooBig>);
static_assert(!WireBody<MidPadding>);
static_assert(!WireBody<TailPadding>);
static_assert(!WireBody<NestedPadding>);
static_assert(!WireBody<ArrayOfPadding>);
static_assert(!WireBody<Pointer>);
static_assert(!WireBody<NestedPointer>);

// 9 bytes is not a CAN-FD length, so it goes out padded to 12
struct NineBytes {
    static constexpr CanId kId{Band::Telemetry, 0x11};
    std::array<std::uint8_t, 9> bytes;
};
static_assert(sizeof(NineBytes) == 9);

// a body whose id names a reserved class
struct ReservedClassBody {
    static constexpr CanId kId{static_cast<Band>(4), 0x10};
    std::uint16_t rpm;
};
}  // namespace

// encode and decode run at compile time (constexpr)
namespace {
constexpr Message kSample{
    .cls        = Band::Command,
    .subaddress = 0x40,
    .body       = {0x01, 0x1E},
    .body_len   = 2,
};
constexpr auto kFrame = encode(kSample);
static_assert(kFrame.has_value());
static_assert(kFrame->id.raw() == 0x140);
static_assert(kFrame->len == 2);
constexpr auto kBack = decode(*kFrame);
static_assert(kBack.has_value());
static_assert(kBack->subaddress == 0x40);
static_assert(kBack->body_len == 2);
}  // namespace

TEST_CASE("encode lays out the id, then the body with no header") {
    const auto f = encode(kSample);
    REQUIRE(f.has_value());
    CHECK(f->id.raw() == 0x140);
    CHECK(f->len == 2);
    CHECK(f->data[0] == 0x01);
    CHECK(f->data[1] == 0x1E);
}

TEST_CASE("decode round-trips encode") {
    const Message m{
        .cls        = Band::Telemetry,
        .subaddress = 0x10,
        .body       = {0xAB},
        .body_len   = 1,
    };
    const auto f = encode(m);
    REQUIRE(f.has_value());
    const auto out = decode(*f);
    REQUIRE(out.has_value());
    CHECK(out->cls == Band::Telemetry);
    CHECK(out->subaddress == 0x10);
    REQUIRE(out->body_len == 1);
    CHECK(out->body[0] == 0xAB);
}

TEST_CASE("decode keeps the whole frame, CAN-FD padding included") {
    CanFrame f{};
    f.id      = CanId(Band::Telemetry, 0x10);
    f.data[0] = 0xAA;
    f.data[1] = 0xBB;
    f.data[2] = 0xCC;
    f.len     = 12;  // the tail past the 3 set bytes is padding

    const auto out = decode(f);
    REQUIRE(out.has_value());
    REQUIRE(out->body_len == 12);
    CHECK(out->body[0] == 0xAA);
    CHECK(out->body[2] == 0xCC);
    CHECK(out->body[11] == 0);
}

TEST_CASE("encode rounds a non-DLC length up to a valid CAN-FD size") {
    // 9 body bytes -> not a valid DLC, rounds to 12
    const Message m{.cls = Band::Telemetry, .subaddress = 0x01, .body_len = 9};
    const auto f = encode(m);
    REQUIRE(f.has_value());
    CHECK(f->len == 12);
    // there is no length byte, so decode sees the padded length
    const auto out = decode(*f);
    REQUIRE(out.has_value());
    CHECK(out->body_len == 12);
}

TEST_CASE("make_stop is a severity stop carrying sender and reason") {
    const auto stop = make_stop(3, 0x12, StopReason::NodeFault);
    const auto f    = encode(stop);
    REQUIRE(f.has_value());
    CHECK(f->id == CanId(Band::Emergency, 3));  // severity 3 -> id 0x003
    CHECK(f->len == 2);
    CHECK(f->data[0] == 0x12);
    CHECK(f->data[1] == std::to_underlying(StopReason::NodeFault));

    const auto view = read_stop(stop);
    REQUIRE(view.has_value());
    CHECK(view->severity == 3);
    CHECK(view->sender == 0x12);
    CHECK(view->reason == StopReason::NodeFault);
}

TEST_CASE("read_stop rejects an emergency frame without the 2-byte body") {
    const Message empty{.cls = Band::Emergency};
    CHECK_FALSE(read_stop(empty).has_value());
    const Message too_long{.cls = Band::Emergency, .body_len = 3};
    CHECK_FALSE(read_stop(too_long).has_value());
}

TEST_CASE("read_stop needs the emergency class") {
    const Message wrong_class{.cls = Band::Telemetry, .body_len = 2};
    CHECK_FALSE(read_stop(wrong_class).has_value());
}

TEST_CASE("read_stop rejects a message that is not a STOP") {
    const auto m = make_message(WheelSpeed{1});
    REQUIRE(m.has_value());
    CHECK_FALSE(read_stop(*m).has_value());
}

TEST_CASE("an empty body encodes to a zero-length frame") {
    const Message m{
        .cls        = Band::Command,
        .subaddress = 0x40,
    };
    const auto f = encode(m);
    REQUIRE(f.has_value());
    CHECK(f->id == CanId(Band::Command, 0x40));
    CHECK(f->len == 0);
}

TEST_CASE("an over-long body is rejected on encode") {
    const Message m{.body_len = kMaxBody + 1};
    CHECK_FALSE(encode(m).has_value());
}

TEST_CASE("make_message rejects an id with an invalid class") {
    CHECK_FALSE(make_message(ReservedClassBody{1}).has_value());
}

TEST_CASE("encode rejects an invalid class") {
    const Message bad_class{.cls = static_cast<Band>(4)};
    CHECK_FALSE(encode(bad_class).has_value());
}

TEST_CASE("a full 64-byte body fills a 64-byte frame and comes back whole") {
    Message m{.cls = Band::Telemetry, .subaddress = 0x10};
    m.body.fill(0x5A);
    m.body_len   = kMaxBody;
    const auto f = encode(m);
    REQUIRE(f.has_value());
    CHECK(f->len == kMaxFdPayload);
    const auto back = decode(*f);
    REQUIRE(back.has_value());
    CHECK(back->body_len == kMaxBody);
    CHECK(back->body[kMaxBody - 1] == 0x5A);
}

TEST_CASE("CAN-FD lengths round up at each size boundary") {
    CHECK(round_up_fd_length(0) == 0);
    CHECK(round_up_fd_length(8) == 8);
    CHECK(round_up_fd_length(9) == 12);
    CHECK(round_up_fd_length(12) == 12);
    CHECK(round_up_fd_length(13) == 16);
    CHECK(round_up_fd_length(48) == 48);
    CHECK(round_up_fd_length(49) == 64);
    CHECK(round_up_fd_length(64) == 64);
    CHECK(is_valid_fd_len(64));
    CHECK_FALSE(is_valid_fd_len(13));
    CHECK_FALSE(is_valid_fd_len(65));
}

TEST_CASE("decode rejects an id outside the 11-bit standard range") {
    CanFrame f{};
    f.id  = CanId::from_raw(0x800);  // would otherwise alias to Emergency/STOP via the class mask
    f.len = 2;
    CHECK(decode(f) == std::unexpected(RxError::NonStandardId));
}

TEST_CASE("decode rejects a reserved class") {
    CanFrame f{};
    f.id  = CanId::from_raw(0x400);
    f.len = 2;
    CHECK(decode(f) == std::unexpected(RxError::UnknownClass));
}

TEST_CASE("decode rejects a len that is not a CAN-FD size") {
    CanFrame f{};
    f.len = 9;
    CHECK(decode(f) == std::unexpected(RxError::BadLength));
    f.len = 13;
    CHECK(decode(f) == std::unexpected(RxError::BadLength));
}

TEST_CASE("body_view clamps a corrupt body_len instead of reading out of bounds") {
    Message m{};
    m.body_len = 200;  // past the 64-byte buffer
    CHECK(m.body_view().size() == kMaxBody);
}

TEST_CASE("decode rejects a nonsensical len past the FD payload size") {
    CanFrame f{};
    f.len = 255;
    CHECK(decode(f) == std::unexpected(RxError::BadLength));
}

TEST_CASE("a struct body round-trips through make_message and body_as") {
    const auto m = make_message(WheelSpeed{1337});
    REQUIRE(m.has_value());
    REQUIRE(m->body_len == 2);
    // 1337 == 0x0539, little-endian on the wire
    CHECK(m->body[0] == 0x39);
    CHECK(m->body[1] == 0x05);

    const auto w = body_as<WheelSpeed>(*m);
    REQUIRE(w.has_value());
    CHECK(w->rpm == 1337);
}

TEST_CASE("body_as rejects a size mismatch") {
    const Message m{.cls = Band::Telemetry, .subaddress = 0x10, .body = {0xAB}, .body_len = 1};
    CHECK_FALSE(body_as<WheelSpeed>(m).has_value());  // body is 1 byte, wants 2
}

TEST_CASE("body_as rejects a body larger than the struct") {
    const Message m{
        .cls        = Band::Telemetry,
        .subaddress = 0x10,
        .body       = {0x01, 0x02, 0x03},
        .body_len   = 3,
    };
    CHECK_FALSE(body_as<WheelSpeed>(m).has_value());
}

TEST_CASE("body_as rejects the right length on another id") {
    const Message m{
        .cls        = Band::Telemetry,
        .subaddress = 0x20,
        .body       = {0x01, 0x02},
        .body_len   = 2,
    };
    CHECK_FALSE(body_as<WheelSpeed>(m).has_value());  // WheelSpeed is 0x210, this is 0x220
}

TEST_CASE("the two 6-byte aux telemetry bodies do not decode as each other") {
    static_assert(sizeof(front_aux::Telemetry) == sizeof(back_aux::Telemetry));
    const auto front = decode(*encode(*make_message(front_aux::Telemetry{1, 2, 3})));
    const auto back  = decode(*encode(*make_message(back_aux::Telemetry{4, 5, 6})));
    REQUIRE(front.has_value());
    REQUIRE(back.has_value());

    CHECK(body_as<front_aux::Telemetry>(*front).has_value());
    CHECK_FALSE(body_as<back_aux::Telemetry>(*front).has_value());
    CHECK(body_as<back_aux::Telemetry>(*back).has_value());
    CHECK_FALSE(body_as<front_aux::Telemetry>(*back).has_value());
}

TEST_CASE("body_as accepts a body padded up to a CAN-FD length") {
    const auto m = make_message(NineBytes{{1, 2, 3, 4, 5, 6, 7, 8, 9}});
    REQUIRE(m.has_value());
    CHECK(m->body_len == 12);
    const auto back = decode(*encode(*m));
    REQUIRE(back.has_value());
    const auto n = body_as<NineBytes>(*back);
    REQUIRE(n.has_value());
    CHECK(n->bytes[8] == 9);
}

TEST_CASE("struct make_message and body_as are constexpr") {
    constexpr auto m = make_message(WheelSpeed{42});
    static_assert(m.has_value());
    static_assert(body_as<WheelSpeed>(*m)->rpm == 42);
    CHECK(true);
}

TEST_CASE("body state goes out as a 1-byte broadcast command on 0x108") {
    const BodyState s{static_cast<std::uint8_t>(BodyState::kHeadlights | BodyState::kLeftTurn)};
    const auto m = make_message(s);
    REQUIRE(m.has_value());
    const auto f = encode(*m);
    REQUIRE(f.has_value());
    CHECK(f->id.raw() == 0x108);
    CHECK(f->len == 1);
    CHECK(f->data[0] == 0x05);
}
