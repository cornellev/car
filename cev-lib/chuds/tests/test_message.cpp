#include <doctest/doctest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string_view>
#include <utility>

#include "chuds/catalog.hpp"
#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/message.hpp"
#include "chuds/transport.hpp"

using namespace chuds;

namespace {
// true when read_stop accepts an argument of type M
template <class M>
concept CanReadStop = requires(M&& m) { read_stop(std::forward<M>(m)); };

// a temporary message would leave the detail span dangling, so it must not compile
static_assert(CanReadStop<const Message&>);
static_assert(!CanReadStop<Message>);

struct WheelSpeed {
    std::uint16_t rpm;
    static constexpr bool is_chuds_wire_body = true;
};
static_assert(sizeof(WheelSpeed) == 2);

enum class Mode : std::uint8_t { Off, On };
struct Mixed {
    std::uint32_t a{};
    std::int16_t b{};
    Mode mode{};
    std::uint8_t c{};
    float d{};
    static constexpr bool is_chuds_wire_body = true;
};
struct Pair {
    std::uint16_t a{};
    std::uint16_t b{};
};
struct Nested {
    Pair pair{};
    std::uint32_t c{};
    static constexpr bool is_chuds_wire_body = true;
};
struct Array {
    std::array<std::uint16_t, 3> values{};
    std::uint16_t d{};
    static constexpr bool is_chuds_wire_body = true;
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
    static constexpr bool is_chuds_wire_body = true;
    NotTriviallyCopyable()                   = default;
    // NOLINTNEXTLINE(modernize-use-equals-default)
    NotTriviallyCopyable(const NotTriviallyCopyable& other) : a(other.a) {}
};
class NotStandardLayout {
   public:
    std::uint16_t a{};
    static constexpr bool is_chuds_wire_body = true;

   private:
    [[maybe_unused]] std::uint16_t b{};
};
struct TooBig {
    std::array<std::uint8_t, kMaxBody + 1> bytes{};
    static constexpr bool is_chuds_wire_body = true;
};
struct MidPadding {
    std::uint8_t a{};
    float b{};
    static constexpr bool is_chuds_wire_body = true;
};
struct TailPadding {
    float a{};
    std::uint8_t b{};
    static constexpr bool is_chuds_wire_body = true;
};
struct NestedPadding {
    MidPadding inner{};
    std::uint32_t c{};
    static constexpr bool is_chuds_wire_body = true;
};
struct ArrayOfPadding {
    std::array<MidPadding, 2> values{};
    static constexpr bool is_chuds_wire_body = true;
};
struct Pointer {
    const char* text{};
    static constexpr bool is_chuds_wire_body = true;
};
struct NestedPointer {
    Pointer inner{};
    static constexpr bool is_chuds_wire_body = true;
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
}  // namespace

// encode and decode run at compile time (constexpr)
namespace {
constexpr Message kSample{
    .cls        = MsgClass::Command,
    .subaddress = 0x40,
    .type       = MsgType::Update,
    .body       = {0x01, 0x1E},
    .body_len   = 2,
};
constexpr auto kFrame = encode(kSample);
static_assert(kFrame.has_value());
static_assert(kFrame->id.raw() == 0x140);
static_assert(kFrame->len == 4);
constexpr auto kBack = decode(*kFrame);
static_assert(kBack.has_value());
static_assert(kBack->subaddress == 0x40);
static_assert(kBack->body_len == 2);
}  // namespace

TEST_CASE("encode lays out id, type, length, then body") {
    const auto f = encode(kSample);
    REQUIRE(f.has_value());
    CHECK(f->id.raw() == 0x140);
    CHECK(f->len == 4);
    CHECK(f->data[0] == std::to_underlying(MsgType::Update));
    CHECK(f->data[1] == 2);
    CHECK(f->data[2] == 0x01);
    CHECK(f->data[3] == 0x1E);
}

TEST_CASE("decode round-trips encode") {
    const Message m{
        .cls        = MsgClass::Telemetry,
        .subaddress = 0x10,
        .type       = MsgType::Action,
        .body       = {0xAB},
        .body_len   = 1,
    };
    const auto f = encode(m);
    REQUIRE(f.has_value());
    const auto out = decode(*f);
    REQUIRE(out.has_value());
    CHECK(out->cls == MsgClass::Telemetry);
    CHECK(out->subaddress == 0x10);
    CHECK(out->type == MsgType::Action);
    REQUIRE(out->body_len == 1);
    CHECK(out->body[0] == 0xAB);
}

TEST_CASE("decode recovers the true body length past CAN-FD padding") {
    CanFrame f{};
    f.id      = CanId(MsgClass::Telemetry, 0x10);
    f.data[0] = std::to_underlying(MsgType::Update);
    f.data[1] = 3;  // true body length
    f.data[2] = 0xAA;
    f.data[3] = 0xBB;
    f.data[4] = 0xCC;
    f.data[5] = 0xFF;  // padding
    f.data[6] = 0xFF;
    f.data[7] = 0xFF;
    f.len     = 8;

    const auto out = decode(f);
    REQUIRE(out.has_value());
    REQUIRE(out->body_len == 3);
    CHECK(out->body[0] == 0xAA);
    CHECK(out->body[2] == 0xCC);
}

TEST_CASE("encode rounds a non-DLC length up to a valid CAN-FD size") {
    // 9 body bytes -> payload 11 -> not a valid DLC, rounds to 12
    const Message m{.cls = MsgClass::Telemetry, .subaddress = 0x01, .body_len = 9};
    const auto f = encode(m);
    REQUIRE(f.has_value());
    CHECK(f->len == 12);
    // decode still recovers the true body length from the length byte
    const auto out = decode(*f);
    REQUIRE(out.has_value());
    CHECK(out->body_len == 9);
}

TEST_CASE("make_stop is a severity stop carrying sender and a detail string") {
    const auto stop = make_stop(0, 0x12, "brake fault");
    const auto f    = encode(stop);
    REQUIRE(f.has_value());
    CHECK(f->id == CanId(MsgClass::Emergency, 0));  // severity 0 -> id 0x000

    const auto view = read_stop(stop);
    REQUIRE(view.has_value());
    CHECK(view->severity == 0);
    CHECK(view->sender == 0x12);
    REQUIRE(view->detail.size() == 11);
    CHECK(view->detail[0] == static_cast<std::uint8_t>('b'));
}

TEST_CASE("make_stop truncates an over-long detail but still transmits") {
    std::array<char, 200> big{};
    big.fill('x');
    const auto stop = make_stop(3, 0x40, std::string_view(big.data(), big.size()));
    CHECK(stop.type == MsgType::Stop);
    CHECK(stop.subaddress == 3);       // severity preserved
    CHECK(stop.body_len == kMaxBody);  // 1 sender + (kMaxBody - 1) detail

    const auto view = read_stop(stop);
    REQUIRE(view.has_value());
    CHECK(view->sender == 0x40);
    CHECK(view->detail.size() == kMaxBody - 1);
}

TEST_CASE("make_stop fills the body exactly at the detail limit") {
    for (const std::size_t len : {kMaxBody - 1, kMaxBody}) {
        std::array<char, kMaxBody> detail{};
        detail.fill('y');
        const auto stop = make_stop(0, 0x12, std::string_view(detail.data(), len));
        CHECK(stop.body_len == kMaxBody);
        const auto view = read_stop(stop);
        REQUIRE(view.has_value());
        CHECK(view->detail.size() == kMaxBody - 1);
    }
}

TEST_CASE("make_stop with an empty detail carries only the sender") {
    const auto stop = make_stop(1, 0x12, "");
    CHECK(stop.body_len == 1);

    const auto view = read_stop(stop);
    REQUIRE(view.has_value());
    CHECK(view->sender == 0x12);
    CHECK(view->detail.empty());
}

TEST_CASE("read_stop rejects a STOP with no sender byte") {
    const Message m{.cls = MsgClass::Emergency, .type = MsgType::Stop};
    CHECK_FALSE(read_stop(m).has_value());
}

TEST_CASE("read_stop needs both the emergency class and the stop type") {
    const Message wrong_type{.cls = MsgClass::Emergency, .type = MsgType::Update, .body_len = 1};
    CHECK_FALSE(read_stop(wrong_type).has_value());
    const Message wrong_class{.cls = MsgClass::Telemetry, .type = MsgType::Stop, .body_len = 1};
    CHECK_FALSE(read_stop(wrong_class).has_value());
}

TEST_CASE("read_stop rejects a message that is not a STOP") {
    const auto m = make_message(MsgClass::Telemetry, 0x10, MsgType::Update, WheelSpeed{1});
    REQUIRE(m.has_value());
    CHECK_FALSE(read_stop(*m).has_value());
}

TEST_CASE("an empty body encodes to the two header bytes") {
    const Message m{
        .cls        = MsgClass::Command,
        .subaddress = 0x40,
        .type       = MsgType::Heartbeat,
    };
    const auto f = encode(m);
    REQUIRE(f.has_value());
    CHECK(f->id == CanId(MsgClass::Command, 0x40));
    CHECK(f->len == 2);
    CHECK(f->data[1] == 0);
}

TEST_CASE("an over-long body is rejected on encode") {
    const Message m{.body_len = kMaxBody + 1};
    CHECK_FALSE(encode(m).has_value());
}

TEST_CASE("make_message rejects an invalid class or type") {
    const WheelSpeed body{1};
    CHECK_FALSE(make_message(static_cast<MsgClass>(3), 0, MsgType::Update, body).has_value());
    CHECK_FALSE(make_message(MsgClass::Telemetry, 0, static_cast<MsgType>(5), body).has_value());
}

TEST_CASE("encode rejects an invalid class or type") {
    const Message bad_class{.cls = static_cast<MsgClass>(3)};
    CHECK_FALSE(encode(bad_class).has_value());
    const Message bad_type{.type = static_cast<MsgType>(5)};
    CHECK_FALSE(encode(bad_type).has_value());
}

TEST_CASE("a full 62-byte body fills a 64-byte frame and comes back whole") {
    Message m{.cls = MsgClass::Telemetry, .subaddress = 0x10, .type = MsgType::Update};
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

TEST_CASE("a 64-byte frame claiming a 63-byte body is an overrun") {
    CanFrame f{};
    f.id      = CanId(MsgClass::Telemetry, 0x10);
    f.data[1] = kMaxBody + 1;
    f.len     = kMaxFdPayload;
    CHECK(decode(f) == std::unexpected(RxError::BodyOverrun));
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

TEST_CASE("a frame shorter than the header is malformed") {
    const CanFrame f{.len = 1};
    CHECK(decode(f) == std::unexpected(RxError::BadLength));
}

TEST_CASE("a length byte larger than the frame is rejected") {
    CanFrame f{};
    f.data[1] = 20;  // claims 20 body bytes
    f.len     = 4;   // but only 2 are present
    CHECK(decode(f) == std::unexpected(RxError::BodyOverrun));
}

TEST_CASE("decode rejects an id outside the 11-bit standard range") {
    CanFrame f{};
    f.id  = CanId::from_raw(0x800);  // would otherwise alias to Emergency/STOP via the class mask
    f.len = 2;
    CHECK(decode(f) == std::unexpected(RxError::NonStandardId));
}

TEST_CASE("decode rejects a reserved class") {
    CanFrame f{};
    f.id  = CanId::from_raw(0x300);
    f.len = 2;
    CHECK(decode(f) == std::unexpected(RxError::UnknownClass));
}

TEST_CASE("decode rejects an unknown type byte") {
    CanFrame f{};
    f.data[0] = 5;
    f.len     = 2;
    CHECK(decode(f) == std::unexpected(RxError::UnknownType));
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
    m.body_len = 200;  // past the 62-byte buffer
    CHECK(m.body_view().size() == kMaxBody);
}

TEST_CASE("decode rejects a nonsensical len past the FD payload size") {
    CanFrame f{};
    f.data[1] = 4;
    f.len     = 255;
    CHECK(decode(f) == std::unexpected(RxError::BadLength));
}

TEST_CASE("a struct body round-trips through make_message and body_as") {
    const auto m = make_message(MsgClass::Telemetry, 0x10, MsgType::Update, WheelSpeed{1337});
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
    const Message m{.cls = MsgClass::Telemetry, .body = {0xAB}, .body_len = 1};
    CHECK_FALSE(body_as<WheelSpeed>(m).has_value());  // body is 1 byte, wants 2
}

TEST_CASE("body_as rejects a body larger than the struct") {
    const Message m{.cls = MsgClass::Telemetry, .body = {0x01, 0x02, 0x03}, .body_len = 3};
    CHECK_FALSE(body_as<WheelSpeed>(m).has_value());
}

TEST_CASE("struct make_message and body_as are constexpr") {
    constexpr auto m = make_message(MsgClass::Telemetry, 0x10, MsgType::Update, WheelSpeed{42});
    static_assert(m.has_value());
    static_assert(body_as<WheelSpeed>(*m)->rpm == 42);
    CHECK(true);
}

TEST_CASE("body state goes out as a 3-byte broadcast command on 0x108") {
    BodyState s{};
    s.set_headlights(true);
    s.set_left_turn(true);
    const auto m = make_message(MsgClass::Command, kBodyStateId, MsgType::Update, s);
    REQUIRE(m.has_value());
    const auto f = encode(*m);
    REQUIRE(f.has_value());
    CHECK(f->id.raw() == 0x108);
    CHECK(f->len == 3);
    CHECK(f->data[0] == std::to_underlying(MsgType::Update));
    CHECK(f->data[1] == 1);
    CHECK(f->data[2] == 0x05);
}
